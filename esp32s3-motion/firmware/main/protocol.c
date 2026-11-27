#include "protocol.h"
#include "motion_config.h"
#include "pid_control.h"
#include "nvs_store.h"

#include <string.h>
#include <math.h>
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "protocol";

#define USB_RX_BUF_SIZE     2048
#define USB_TX_BUF_SIZE     2048
#define TURBO_MAX_PAYLOAD   64

// Streaming telemetry state
static uint8_t  s_smc3_monitor_motor = 0;    // 0=off, 1..3=SMC3 [mo1..3] 20 ms stream
static uint16_t s_turbo_stream_hz    = 0;    // 0=off, 1..250 Hz Turbo v2 binary telemetry
static uint32_t s_last_loop_read     = 0;
static volatile bool s_nvs_save_req  = false;

// ============================================================================
// CRC-8 (polynomial 0x07, SMBus) for Turbo v2 frames
// ============================================================================
static uint8_t crc8_calc(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline void usb_write_all(const uint8_t *buf, size_t len) {
    // Non-blocking / tiny-timeout write so disconnected USB host never hangs Core 0
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(2));
}

// ============================================================================
// SMC3 v0.70 + SimSync 5-byte Frame Helpers
// ============================================================================
static inline uint8_t duty_1000_to_255(int16_t d) {
    int32_t a = (d < 0) ? -d : d;
    int32_t v = (a * 255 + 500) / 1000;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

static inline uint16_t pwm_255_to_1000(uint8_t p) {
    return (uint16_t)(((uint32_t)p * 1000U + 127U) / 255U);
}

static void smc3_send_two(uint8_t id, uint8_t v1, uint8_t v2) {
    uint8_t frame[5] = { '[', id, v1, v2, ']' };
    usb_write_all(frame, sizeof(frame));
}

static void smc3_send_word(uint8_t id, uint16_t val) {
    smc3_send_two(id, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF));
}

static void handle_smc3_read(uint8_t code) {
    telem_snapshot_t snap;
    controller_cfg_t cfg;
    pid_get_snapshot(&snap);
    pid_get_config(&cfg);

    uint8_t status_byte = (uint8_t)((snap.pid_divider << 4) | (snap.disabled_mask & 0x07));

    switch (code) {
        case '!': case '"': case '#': {
            int i = code - '!';
            smc3_send_word(code, (uint16_t)lroundf(cfg.motors[i].lpf_hz * 10.0f));
            break;
        }
        case '$': case '%': case '&': {
            int i = code - '$';
            smc3_send_word(code, (uint16_t)lroundf(cfg.motors[i].hpf_hz * 10.0f));
            break;
        }
        case 'A': case 'B': case 'C': {
            int i = code - 'A';
            // 12-bit >> 4 == 10-bit >> 2 (exact SMC3 0..255 feedback/target scale)
            smc3_send_two(code, (uint8_t)(snap.actual_12[i] >> 4), (uint8_t)(snap.target_12[i] >> 4));
            break;
        }
        case 'a': case 'b': case 'c': {
            int i = code - 'a';
            smc3_send_two(code, status_byte, duty_1000_to_255(snap.duty[i]));
            break;
        }
        case 'D': case 'E': case 'F':
            smc3_send_word(code, (uint16_t)lroundf(cfg.motors[code - 'D'].kp * 100.0f));
            break;
        case 'G': case 'H': case 'I':
            smc3_send_word(code, (uint16_t)lroundf(cfg.motors[code - 'G'].ki * 100.0f));
            break;
        case 'J': case 'K': case 'L':
            smc3_send_word(code, (uint16_t)lroundf(cfg.motors[code - 'J'].kd * 100.0f));
            break;
        case 'M': case 'N': case 'O':
            smc3_send_word(code, cfg.motors[code - 'M'].ks);
            break;
        case 'P': case 'Q': case 'R': {
            int i = code - 'P';
            smc3_send_two(code,
                          duty_1000_to_255((int16_t)cfg.motors[i].pwm_min),
                          duty_1000_to_255((int16_t)cfg.motors[i].pwm_max));
            break;
        }
        case 'S': case 'T': case 'U': {
            int i = code - 'S';
            smc3_send_two(code,
                          (uint8_t)(cfg.motors[i].cutoff_min_12 >> 2),
                          (uint8_t)(cfg.motors[i].clip_min_12 >> 2));
            break;
        }
        case 'V': case 'W': case 'X': {
            int i = code - 'V';
            smc3_send_two(code,
                          (uint8_t)(cfg.motors[i].deadzone_12 >> 2),
                          duty_1000_to_255((int16_t)cfg.motors[i].pwm_rev));
            break;
        }
        case 'Y':
            smc3_send_two('Y', status_byte, 0);
            break;
        case 'Z': {
            uint32_t delta = 0;
            if (s_last_loop_read != 0 && (snap.loop_count - s_last_loop_read) <= 32000UL) {
                delta = snap.loop_count - s_last_loop_read;
            }
            s_last_loop_read = snap.loop_count;
            smc3_send_word('Z', (uint16_t)delta);
            break;
        }
        case '~':
            smc3_send_two('~', (uint8_t)cfg.pwm12_khz, (uint8_t)cfg.pwm3_khz);
            break;
        default:
            break;
    }
}

static void handle_smc3_command(uint8_t cmd, uint8_t b1, uint8_t b2) {
    pid_touch_comms();
    uint16_t word_val = ((uint16_t)b1 << 8) | b2;

    switch (cmd) {
        case 'A': case 'B': case 'C': {
            // Map 10-bit SMC3 target (0..1023) to 12-bit internal target (0..4092)
            uint16_t t10 = (word_val > 1023) ? 1023 : word_val;
            pid_set_target_12(cmd - 'A', (uint16_t)(t10 << 2));
            break;
        }
        case 'D': case 'E': case 'F':
            pid_set_kp(cmd - 'D', (float)word_val * 0.01f);
            break;
        case 'G': case 'H': case 'I':
            pid_set_ki(cmd - 'G', (float)word_val * 0.01f);
            break;
        case 'J': case 'K': case 'L':
            pid_set_kd(cmd - 'J', (float)word_val * 0.01f);
            break;
        case 'M': case 'N': case 'O':
            pid_set_ks(cmd - 'M', word_val < 1 ? 1 : word_val);
            break;
        case 'P': case 'Q': case 'R':
            pid_set_pwm_limits(cmd - 'P', pwm_255_to_1000(b1), pwm_255_to_1000(b2));
            break;
        case 'S': case 'T': case 'U': {
            uint16_t cmin = (uint16_t)b1 << 2;
            uint16_t imin = (uint16_t)b2 << 2;
            pid_set_travel_limits_12(cmd - 'S', cmin, 4095 - cmin, imin, 4095 - imin);
            break;
        }
        case 'V': case 'W': case 'X':
            pid_set_deadzone_rev(cmd - 'V', (uint16_t)b1 << 2, pwm_255_to_1000(b2));
            break;
        case 'Z':
            pid_set_divider(b1);
            break;
        case '~':
            pid_set_pwm_khz(b1, b2);
            break;
        case '!': case '"': case '#':
            if (word_val <= 2000) {
                pid_set_lpf(cmd - '!', (float)word_val * 0.1f);
            }
            break;
        case '$': case '%': case '&':
            if (word_val <= 500) {
                pid_set_hpf(cmd - '$', (float)word_val * 0.1f);
            }
            break;
        case '4': case '5': case '6':
            pid_set_jog_min_12(cmd - '4', (uint16_t)((word_val > 1023 ? 1023 : word_val) << 2));
            break;
        case '7': case '8': case '9':
            pid_set_jog_max_12(cmd - '7', (uint16_t)((word_val > 1023 ? 1023 : word_val) << 2));
            break;
        case 'j': case 'k': case 'l': {
            int8_t signed_pwm = (int8_t)b1;
            int16_t duty_1000 = (int16_t)(((int32_t)signed_pwm * 1000) / 255);
            pid_start_jog(cmd - 'j', duty_1000, (uint16_t)b2 * 10U);
            break;
        }
        case 'h':
            if (b1 == 0) pid_hold_mask(b2);
            break;
        case 'c':
            if (b1 == 'a' && b2 == 'p') smc3_send_word('q', SMC3_CAPABILITY_FLAGS);
            break;
        case 'v':
            if (b1 == 'e' && b2 == 'r') smc3_send_word('v', SMC3_FIRMWARE_VERSION);
            break;
        case 'r':
            if (b1 == 'd') handle_smc3_read(b2);
            break;
        case 'm':
            if (b1 == 'o' && b2 >= '0' && b2 <= '3') {
                s_smc3_monitor_motor = b2 - '0';
            }
            break;
        case 's':
            if (b1 == 't' && b2 == 'p') {
                pid_disable_all();
            } else if (b1 == 'a' && b2 == 'v') {
                s_nvs_save_req = true;
            }
            break;
        case 'e':
            if (b1 == 'n') {
                if (b2 == 'a') {
                    pid_enable_mask(0x07);
                } else if (b2 >= '1' && b2 <= '3') {
                    pid_enable_motor(b2 - '1');
                }
            }
            break;
        default:
            break;
    }
}

// ============================================================================
// Native ESP32-S3 Turbo v2 Binary Frame Helpers
// Wire format: [0xAA, 0x55, type, len, payload[len], crc8(type..payload)]
// ============================================================================
static void turbo_send_packet(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t pkt[4 + TURBO_MAX_PAYLOAD + 1];
    if (len > TURBO_MAX_PAYLOAD) return;
    pkt[0] = TURBO_SYNC_0;
    pkt[1] = TURBO_SYNC_1;
    pkt[2] = type;
    pkt[3] = len;
    if (len > 0 && payload != NULL) {
        memcpy(&pkt[4], payload, len);
    }
    pkt[4 + len] = crc8_calc(&pkt[2], 2 + len);
    usb_write_all(pkt, 5 + len);
}

static inline uint16_t rd_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline float rd_f32_le(const uint8_t *p) {
    float v;
    memcpy(&v, p, sizeof(v));
    return v;
}

static inline void wr_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline void wr_i16_le(uint8_t *p, int16_t v) {
    wr_u16_le(p, (uint16_t)v);
}

static inline void wr_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void wr_f32_le(uint8_t *p, float v) {
    memcpy(p, &v, sizeof(v));
}

static void turbo_send_telemetry(void) {
    telem_snapshot_t s;
    pid_get_snapshot(&s);

    uint8_t buf[50];
    size_t off = 0;
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        wr_u16_le(&buf[off], s.actual_12[i]);      off += 2;
        wr_u16_le(&buf[off], s.target_12[i]);      off += 2;
        wr_i16_le(&buf[off], s.duty[i]);           off += 2;
        wr_u16_le(&buf[off], s.noise_p2p_12[i]);   off += 2;
        wr_u16_le(&buf[off], s.noise_rms_x100[i]); off += 2;
        wr_u16_le(&buf[off], s.adc_samples[i]);    off += 2;
    }
    buf[off++] = s.disabled_mask;
    buf[off++] = s.fault_flags;
    buf[off++] = s.active_dof;
    buf[off++] = s.pid_divider;
    wr_u16_le(&buf[off], s.loop_exec_us);   off += 2;
    wr_u16_le(&buf[off], s.loop_max_us);    off += 2;
    wr_u16_le(&buf[off], s.loop_jitter_us); off += 2;
    wr_u32_le(&buf[off], s.loop_count);     off += 4;

    turbo_send_packet(TURBO_PKT_TELEMETRY, buf, (uint8_t)off);
}

static void turbo_send_config(uint8_t which) {
    controller_cfg_t cfg;
    pid_get_config(&cfg);

    if (which == 0xFF) {
        uint8_t buf[10];
        buf[0] = cfg.active_dof;
        buf[1] = cfg.pid_divider;
        wr_u16_le(&buf[2], cfg.pwm12_khz);
        wr_u16_le(&buf[4], cfg.pwm3_khz);
        wr_u16_le(&buf[6], CONFIG_SIMSYNC_PID_FREQ_HZ);
        wr_u16_le(&buf[8], (uint16_t)(CONFIG_SIMSYNC_ADC_SAMPLE_RATE_HZ / 1000));
        turbo_send_packet(TURBO_PKT_GLOBAL_CFG, buf, sizeof(buf));
        return;
    }

    if (which < SIMSYNC_NUM_MOTORS) {
        const motor_cfg_t *m = &cfg.motors[which];
        uint8_t buf[45];
        size_t off = 0;
        buf[off++] = which;
        wr_f32_le(&buf[off], m->kp);            off += 4;
        wr_f32_le(&buf[off], m->ki);            off += 4;
        wr_f32_le(&buf[off], m->kd);            off += 4;
        wr_f32_le(&buf[off], m->kff);           off += 4;
        wr_f32_le(&buf[off], m->lpf_hz);        off += 4;
        wr_f32_le(&buf[off], m->hpf_hz);        off += 4;
        wr_u16_le(&buf[off], m->ks);            off += 2;
        wr_u16_le(&buf[off], m->pwm_min);       off += 2;
        wr_u16_le(&buf[off], m->pwm_max);       off += 2;
        wr_u16_le(&buf[off], m->pwm_rev);       off += 2;
        wr_u16_le(&buf[off], m->deadzone_12);   off += 2;
        wr_u16_le(&buf[off], m->cutoff_min_12); off += 2;
        wr_u16_le(&buf[off], m->cutoff_max_12); off += 2;
        wr_u16_le(&buf[off], m->clip_min_12);   off += 2;
        wr_u16_le(&buf[off], m->clip_max_12);   off += 2;
        buf[off++] = m->invert_pot;
        buf[off++] = m->invert_dir;
        turbo_send_packet(TURBO_PKT_MOTOR_CFG, buf, (uint8_t)off);
    }
}

static void handle_turbo_command(uint8_t type, const uint8_t *pl, uint8_t len) {
    pid_touch_comms();
    uint8_t ack_status = 0;

    switch (type) {
        case TURBO_CMD_TARGETS_12:
            if (len == 7) {
                uint16_t t[3] = {
                    rd_u16_le(&pl[0]),
                    rd_u16_le(&pl[2]),
                    rd_u16_le(&pl[4]),
                };
                pid_set_targets_batch_12(t, pl[6]);
            }
            return; // No ACK on 1000 Hz target stream!

        case TURBO_CMD_SET_GAINS:
            if (len == 27 && pl[0] < SIMSYNC_NUM_MOTORS) {
                pid_set_gains(
                    pl[0],
                    rd_f32_le(&pl[1]),
                    rd_f32_le(&pl[5]),
                    rd_f32_le(&pl[9]),
                    rd_f32_le(&pl[13]),
                    rd_u16_le(&pl[25]),
                    rd_f32_le(&pl[17]),
                    rd_f32_le(&pl[21]));
            } else {
                ack_status = 1;
            }
            break;

        case TURBO_CMD_SET_LIMITS:
            if (len == 19 && pl[0] < SIMSYNC_NUM_MOTORS) {
                int m = pl[0];
                pid_set_travel_limits_12(m,
                    rd_u16_le(&pl[1]), rd_u16_le(&pl[3]),
                    rd_u16_le(&pl[5]), rd_u16_le(&pl[7]));
                pid_set_deadzone_rev(m, rd_u16_le(&pl[9]), rd_u16_le(&pl[15]));
                pid_set_pwm_limits(m, rd_u16_le(&pl[11]), rd_u16_le(&pl[13]));
                pid_set_polarity(m, pl[17] != 0, pl[18] != 0);
            } else {
                ack_status = 1;
            }
            break;

        case TURBO_CMD_CONTROL:
            if (len == 5) {
                uint8_t  op   = pl[0];
                uint16_t arg1 = rd_u16_le(&pl[1]);
                uint16_t arg2 = rd_u16_le(&pl[3]);
                switch (op) {
                    case TURBO_OP_STOP_ALL:       pid_disable_all(); break;
                    case TURBO_OP_ENABLE_MASK:    pid_enable_mask((uint8_t)arg1); break;
                    case TURBO_OP_HOLD_MASK:      pid_hold_mask((uint8_t)arg1); break;
                    case TURBO_OP_CLEAR_ESTOP:    ack_status = pid_clear_estop() ? 0 : 2; break;
                    case TURBO_OP_SAVE_NVS:       s_nvs_save_req = true; break;
                    case TURBO_OP_SET_STREAM_HZ:  s_turbo_stream_hz = (arg1 > 250) ? 250 : arg1; break;
                    case TURBO_OP_SET_PWM_KHZ:    pid_set_pwm_khz(arg1, arg2); break;
                    case TURBO_OP_SET_ACTIVE_DOF: pid_set_active_dof((uint8_t)arg1); break;
                    case TURBO_OP_RESET_STATS:    pid_reset_timing_stats(); break;
                    default:                      ack_status = 1; break;
                }
            } else {
                ack_status = 1;
            }
            break;

        case TURBO_CMD_GET_CONFIG:
            if (len == 1) {
                turbo_send_config(pl[0]);
                return;
            }
            ack_status = 1;
            break;

        default:
            ack_status = 1;
            break;
    }

    uint8_t ack[2] = { type, ack_status };
    turbo_send_packet(TURBO_PKT_ACK, ack, sizeof(ack));
}

// ============================================================================
// Auto-Multiplexed Stream Parser (SMC3 '['..']' + Turbo v2 0xAA 0x55)
// ============================================================================
typedef enum {
    PARSE_IDLE = 0,
    PARSE_SMC3,
    PARSE_TURBO_SYNC1,
    PARSE_TURBO_TYPE,
    PARSE_TURBO_LEN,
    PARSE_TURBO_PAYLOAD,
    PARSE_TURBO_CRC,
} parse_state_t;

esp_err_t protocol_init(void) {
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = USB_RX_BUF_SIZE,
        .tx_buffer_size = USB_TX_BUF_SIZE,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Native USB Serial/JTAG ready (SMC3 v0.70 + Turbo v2 12-bit binary)");
    return ESP_OK;
}

void protocol_task(void *arg) {
    (void)arg;

    uint8_t rx_chunk[128];
    parse_state_t state = PARSE_IDLE;

    // SMC3 frame buffer
    uint8_t smc3_buf[4] = {0};
    int     smc3_idx    = 0;

    // Turbo v2 frame buffer
    uint8_t turbo_hdr[2] = {0}; // [0]=type, [1]=len
    uint8_t turbo_pl[TURBO_MAX_PAYLOAD];
    uint8_t turbo_idx = 0;

    int64_t last_smc3_mon_us = esp_timer_get_time();
    int64_t last_turbo_tx_us = esp_timer_get_time();

    while (1) {
        // 1 ms timeout keeps USB polling sub-ms while yielding cleanly to Core 0 idle/WDT
        int n = usb_serial_jtag_read_bytes(rx_chunk, sizeof(rx_chunk), pdMS_TO_TICKS(1));
        for (int k = 0; k < n; ++k) {
            uint8_t b = rx_chunk[k];
            switch (state) {
                case PARSE_IDLE:
                    if (b == '[') {
                        smc3_idx = 0;
                        state = PARSE_SMC3;
                    } else if (b == TURBO_SYNC_0) {
                        state = PARSE_TURBO_SYNC1;
                    }
                    break;

                case PARSE_SMC3:
                    smc3_buf[smc3_idx++] = b;
                    if (smc3_idx == 4) {
                        if (smc3_buf[3] == ']') {
                            handle_smc3_command(smc3_buf[0], smc3_buf[1], smc3_buf[2]);
                        }
                        state = PARSE_IDLE;
                    }
                    break;

                case PARSE_TURBO_SYNC1:
                    if (b == TURBO_SYNC_1) {
                        state = PARSE_TURBO_TYPE;
                    } else if (b == '[') {
                        smc3_idx = 0;
                        state = PARSE_SMC3;
                    } else {
                        state = PARSE_IDLE;
                    }
                    break;

                case PARSE_TURBO_TYPE:
                    turbo_hdr[0] = b;
                    state = PARSE_TURBO_LEN;
                    break;

                case PARSE_TURBO_LEN:
                    turbo_hdr[1] = b;
                    if (b > TURBO_MAX_PAYLOAD) {
                        state = PARSE_IDLE;
                    } else if (b == 0) {
                        state = PARSE_TURBO_CRC;
                    } else {
                        turbo_idx = 0;
                        state = PARSE_TURBO_PAYLOAD;
                    }
                    break;

                case PARSE_TURBO_PAYLOAD:
                    turbo_pl[turbo_idx++] = b;
                    if (turbo_idx >= turbo_hdr[1]) {
                        state = PARSE_TURBO_CRC;
                    }
                    break;

                case PARSE_TURBO_CRC: {
                    uint8_t check_buf[2 + TURBO_MAX_PAYLOAD];
                    check_buf[0] = turbo_hdr[0];
                    check_buf[1] = turbo_hdr[1];
                    if (turbo_hdr[1] > 0) {
                        memcpy(&check_buf[2], turbo_pl, turbo_hdr[1]);
                    }
                    uint8_t expected = crc8_calc(check_buf, 2 + turbo_hdr[1]);
                    if (b == expected) {
                        handle_turbo_command(turbo_hdr[0], turbo_pl, turbo_hdr[1]);
                    }
                    state = PARSE_IDLE;
                    break;
                }
            }
        }

        // Service deferred NVS save safely on Core 0
        if (s_nvs_save_req) {
            s_nvs_save_req = false;
            controller_cfg_t cfg;
            pid_get_config(&cfg);
            nvs_store_save(&cfg);
        }

        int64_t now_us = esp_timer_get_time();

        // Optional SMC3 [mo1/2/3] 20 ms monitor stream
        if (s_smc3_monitor_motor >= 1 && s_smc3_monitor_motor <= SIMSYNC_NUM_MOTORS) {
            if ((now_us - last_smc3_mon_us) >= 20000LL) {
                last_smc3_mon_us = now_us;
                telem_snapshot_t s;
                pid_get_snapshot(&s);
                int i = s_smc3_monitor_motor - 1;
                uint8_t status_byte = (uint8_t)((s.pid_divider << 4) | (s.disabled_mask & 0x07));
                smc3_send_two((uint8_t)('A' + i), (uint8_t)(s.actual_12[i] >> 4), (uint8_t)(s.target_12[i] >> 4));
                smc3_send_two((uint8_t)('a' + i), status_byte, duty_1000_to_255(s.duty[i]));
            }
        }

        // Optional Turbo v2 high-res telemetry stream (up to 250 Hz)
        if (s_turbo_stream_hz > 0) {
            int64_t period_us = 1000000LL / (int64_t)s_turbo_stream_hz;
            if ((now_us - last_turbo_tx_us) >= period_us) {
                last_turbo_tx_us = now_us;
                turbo_send_telemetry();
            }
        }
    }
}
