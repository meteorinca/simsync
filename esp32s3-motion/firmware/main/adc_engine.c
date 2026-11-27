#include "adc_engine.h"

#include <string.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_filter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "adc_engine";

// At 72 kSPS total across 3 channels = 24 kSPS/ch -> 12 samples/ch per 500 us (2 kHz) tick.
// Each conversion result on ESP32-S3 is 4 bytes (SOC_ADC_DIGI_DATA_BYTES_PER_CONV = 4).
// 36 conversions = 144 bytes per DMA frame -> fires every 500 us seamlessly!
#define ADC_FRAME_BYTES     144
#define ADC_POOL_BYTES      (ADC_FRAME_BYTES * 8)
#define ADC_READ_BUF_BYTES  (ADC_FRAME_BYTES * 4)
#define ADC_MAX_PER_CH      64

// ESP32-S3 ADC_ATTEN_DB_12 accurate calibrated full-scale voltage (mV)
#define ADC_FULL_SCALE_MV   3150.0f

static adc_continuous_handle_t s_adc_handle = NULL;
static adc_cali_handle_t       s_cali_handle[SIMSYNC_NUM_MOTORS] = {NULL};
static bool                    s_cali_ok[SIMSYNC_NUM_MOTORS] = {false};

#if CONFIG_SIMSYNC_ADC_HW_IIR_2DOF
static adc_iir_filter_handle_t s_iir_handle[2] = {NULL, NULL};
#endif

static const adc_channel_t s_motor_channels[SIMSYNC_NUM_MOTORS] = {
    (adc_channel_t)CONFIG_SIMSYNC_M1_ADC_CH,
    (adc_channel_t)CONFIG_SIMSYNC_M2_ADC_CH,
    (adc_channel_t)CONFIG_SIMSYNC_M3_ADC_CH,
};

// Reverse lookup table: ADC1 channel (0..9) -> motor index (0..2) or -1
static int8_t s_ch_to_motor[10];

// Cached last good values in case a tick fires slightly before a new DMA frame
static float    s_last_val[SIMSYNC_NUM_MOTORS] = {2048.0f, 2048.0f, 2048.0f};
static uint16_t s_last_p2p[SIMSYNC_NUM_MOTORS] = {0, 0, 0};

// DMA overflow flag set from ISR
static volatile bool s_pool_ovf = false;

static bool IRAM_ATTR on_pool_ovf_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
    (void)handle;
    (void)edata;
    (void)user_data;
    s_pool_ovf = true;
    return false;
}

static inline float raw_to_calibrated_12bit(int motor_idx, int raw) {
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;

    if (s_cali_ok[motor_idx]) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali_handle[motor_idx], raw, &mv) == ESP_OK) {
            float scaled = ((float)mv / ADC_FULL_SCALE_MV) * 4095.0f;
            if (scaled < 0.0f) scaled = 0.0f;
            if (scaled > 4095.0f) scaled = 4095.0f;
            return scaled;
        }
    }
    return (float)raw;
}

esp_err_t adc_engine_init(void) {
    for (int i = 0; i < 10; ++i) {
        s_ch_to_motor[i] = -1;
    }
    for (int m = 0; m < SIMSYNC_NUM_MOTORS; ++m) {
        s_ch_to_motor[(int)s_motor_channels[m]] = (int8_t)m;
    }

    // 1. Initialize eFuse curve-fitting calibration per channel
    for (int m = 0; m < SIMSYNC_NUM_MOTORS; ++m) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .chan     = s_motor_channels[m],
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle[m]);
        if (err == ESP_OK) {
            s_cali_ok[m] = true;
            ESP_LOGI(TAG, "M%d ADC1_CH%d: curve-fitting eFuse calibration active",
                     m + 1, (int)s_motor_channels[m]);
        } else {
            s_cali_ok[m] = false;
            ESP_LOGW(TAG, "M%d ADC1_CH%d: eFuse calibration unavailable (%s), using raw 12-bit",
                     m + 1, (int)s_motor_channels[m], esp_err_to_name(err));
        }
#endif
    }

    // 2. Create continuous GDMA ADC handle with flush_pool so freshest samples always win
    adc_continuous_handle_cfg_t hdl_cfg = {
        .max_store_buf_size = ADC_POOL_BYTES,
        .conv_frame_size    = ADC_FRAME_BYTES,
        .flags = {
            .flush_pool = 1,
        },
    };
    esp_err_t err = adc_continuous_new_handle(&hdl_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_new_handle failed: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Configure ADC1 3-channel scan pattern (12-bit, 12 dB attenuation -> 0..3.15V)
    adc_digi_pattern_config_t patterns[SIMSYNC_NUM_MOTORS];
    for (int m = 0; m < SIMSYNC_NUM_MOTORS; ++m) {
        patterns[m].atten     = ADC_ATTEN_DB_12;
        patterns[m].channel   = s_motor_channels[m] & 0x7F;
        patterns[m].unit      = ADC_UNIT_1;
        patterns[m].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; // 12
    }

    adc_continuous_config_t dig_cfg = {
        .pattern_num    = SIMSYNC_NUM_MOTORS,
        .adc_pattern    = patterns,
        .sample_freq_hz = CONFIG_SIMSYNC_ADC_SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    err = adc_continuous_config(s_adc_handle, &dig_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_config failed: %s", esp_err_to_name(err));
        return err;
    }

    // 4. Optional ESP32-S3 silicon IIR filters for M1 & M2 (2DOF mode)
#if CONFIG_SIMSYNC_ADC_HW_IIR_2DOF
    for (int m = 0; m < 2; ++m) {
        adc_continuous_iir_filter_config_t iir_cfg = {
            .unit    = ADC_UNIT_1,
            .channel = s_motor_channels[m],
            .coeff   = ADC_DIGI_IIR_FILTER_COEFF_4,
        };
        err = adc_new_continuous_iir_filter(s_adc_handle, &iir_cfg, &s_iir_handle[m]);
        if (err == ESP_OK) {
            adc_continuous_iir_filter_enable(s_iir_handle[m]);
            ESP_LOGI(TAG, "M%d hardware silicon IIR filter enabled (coeff=4)", m + 1);
        } else {
            ESP_LOGW(TAG, "M%d hardware IIR filter init failed: %s", m + 1, esp_err_to_name(err));
        }
    }
#endif

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = NULL,
        .on_pool_ovf  = on_pool_ovf_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));

    ESP_LOGI(TAG, "ADC1 GDMA continuous started @ %d Hz (%d Hz/motor)",
             CONFIG_SIMSYNC_ADC_SAMPLE_RATE_HZ,
             CONFIG_SIMSYNC_ADC_SAMPLE_RATE_HZ / SIMSYNC_NUM_MOTORS);
    return ESP_OK;
}

void adc_engine_sample(adc_batch_t *out) {
    static uint8_t dma_buf[ADC_READ_BUF_BYTES];

    uint32_t sum[SIMSYNC_NUM_MOTORS]   = {0};
    uint16_t count[SIMSYNC_NUM_MOTORS] = {0};
    uint16_t vmin[SIMSYNC_NUM_MOTORS]  = {4095, 4095, 4095};
    uint16_t vmax[SIMSYNC_NUM_MOTORS]  = {0, 0, 0};

    // Drain up to 2 calls non-blockingly (timeout_ms = 0) so we always process fresh data
    for (int pass = 0; pass < 2; ++pass) {
        uint32_t out_len = 0;
        esp_err_t err = adc_continuous_read(s_adc_handle, dma_buf, sizeof(dma_buf), &out_len, 0);
        if (err != ESP_OK || out_len == 0) {
            break;
        }
        uint32_t n = out_len / SOC_ADC_DIGI_RESULT_BYTES;
        const adc_digi_output_data_t *items = (const adc_digi_output_data_t *)dma_buf;
        for (uint32_t i = 0; i < n; ++i) {
            uint32_t unit = items[i].type2.unit;
            uint32_t ch   = items[i].type2.channel;
            uint16_t raw  = (uint16_t)items[i].type2.data;
            if (unit != 0 || ch >= 10) {
                continue;
            }
            int8_t m = s_ch_to_motor[ch];
            if (m < 0 || count[m] >= ADC_MAX_PER_CH) {
                continue;
            }
            sum[m] += raw;
            if (raw < vmin[m]) vmin[m] = raw;
            if (raw > vmax[m]) vmax[m] = raw;
            count[m]++;
        }
        if (out_len < sizeof(dma_buf)) {
            break;
        }
    }

    out->overflow = s_pool_ovf;
    s_pool_ovf = false;

    for (int m = 0; m < SIMSYNC_NUM_MOTORS; ++m) {
        out->sample_count[m] = count[m];
        if (count[m] == 0) {
            out->value_12[m] = s_last_val[m];
            out->p2p_12[m]   = s_last_p2p[m];
            continue;
        }

        // Outlier-trimmed mean: when >= 4 samples are collected, discard single highest
        // and single lowest sample to completely reject PWM switching spikes!
        float avg_raw;
        if (count[m] >= 4) {
            uint32_t trimmed_sum = sum[m] - vmin[m] - vmax[m];
            avg_raw = (float)trimmed_sum / (float)(count[m] - 2);
        } else {
            avg_raw = (float)sum[m] / (float)count[m];
        }

        // Convert averaged raw value through curve-fitting calibration with fractional interpolation
        int raw_lo = (int)avg_raw;
        int raw_hi = (raw_lo < 4095) ? (raw_lo + 1) : 4095;
        float frac = avg_raw - (float)raw_lo;
        float cal_lo = raw_to_calibrated_12bit(m, raw_lo);
        float cal_hi = raw_to_calibrated_12bit(m, raw_hi);
        float cal_val = cal_lo + frac * (cal_hi - cal_lo);

        uint16_t p2p = (vmax[m] >= vmin[m]) ? (vmax[m] - vmin[m]) : 0;

        s_last_val[m]    = cal_val;
        s_last_p2p[m]    = p2p;
        out->value_12[m] = cal_val;
        out->p2p_12[m]   = p2p;
    }
}

void adc_engine_warmup(adc_batch_t *out, uint32_t settle_ms) {
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
    adc_engine_sample(out);
    vTaskDelay(pdMS_TO_TICKS(5));
    adc_engine_sample(out);
}
