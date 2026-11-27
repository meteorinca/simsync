#include "mcpwm_drive.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "mcpwm_drive";

// 40 MHz timer clock:
//   At 20 kHz UP-DOWN: period_ticks = 40,000,000 / 20,000 = 2000 -> peak = 1000 steps (0.1% duty!)
//   At 25 kHz UP-DOWN: period_ticks = 1600 -> peak = 800 steps
//   At  1 kHz UP-DOWN: period_ticks = 40000 -> peak = 20000 steps (fits uint16_t <= 65535!)
#define MCPWM_TIMER_RES_HZ      40000000UL

typedef struct {
    int rpwm_gpio;
    int lpwm_gpio;
    int en_gpio;
} motor_pins_t;

static const motor_pins_t s_pins[SIMSYNC_NUM_MOTORS] = {
    { CONFIG_SIMSYNC_M1_RPWM_GPIO, CONFIG_SIMSYNC_M1_LPWM_GPIO, CONFIG_SIMSYNC_M1_EN_GPIO },
    { CONFIG_SIMSYNC_M2_RPWM_GPIO, CONFIG_SIMSYNC_M2_LPWM_GPIO, CONFIG_SIMSYNC_M2_EN_GPIO },
    { CONFIG_SIMSYNC_M3_RPWM_GPIO, CONFIG_SIMSYNC_M3_LPWM_GPIO, CONFIG_SIMSYNC_M3_EN_GPIO },
};

typedef struct {
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr_r;
    mcpwm_cmpr_handle_t cmpr_l;
    mcpwm_gen_handle_t  gen_r;
    mcpwm_gen_handle_t  gen_l;
    int8_t              last_dir;   // -1, 0, +1 (avoids redundant force transitions)
} motor_pwm_hw_t;

static mcpwm_timer_handle_t s_timer_12 = NULL;   // M1 & M2 synchronized center-aligned timer
static mcpwm_timer_handle_t s_timer_3  = NULL;   // M3 independent center-aligned timer
static uint32_t             s_peak_12  = 1000;
static uint32_t             s_peak_3   = 1000;
static motor_pwm_hw_t       s_hw[SIMSYNC_NUM_MOTORS];

#if CONFIG_SIMSYNC_ESTOP_GPIO >= 0
static mcpwm_fault_handle_t s_estop_fault = NULL;
#endif
static volatile bool        s_estop_latched = false;

static uint32_t clamp_khz(uint16_t khz) {
    if (khz < 1) return 1;
    if (khz > 25) return 25;
    return khz;
}

static uint32_t khz_to_period_ticks(uint16_t khz) {
    uint32_t f = clamp_khz(khz) * 1000UL;
    uint32_t ticks = MCPWM_TIMER_RES_HZ / f;
    // UP-DOWN mode expects even period_ticks (peak = period_ticks / 2)
    ticks &= ~1UL;
    if (ticks < 200) ticks = 200;
    if (ticks > 65000) ticks = 65000;
    return ticks;
}

#if CONFIG_SIMSYNC_ESTOP_GPIO >= 0
static bool IRAM_ATTR on_estop_brake_cb(mcpwm_oper_handle_t oper,
                                        const mcpwm_brake_event_data_t *edata,
                                        void *user_data) {
    (void)oper;
    (void)edata;
    (void)user_data;
    s_estop_latched = true;
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        gpio_set_level(s_pins[i].en_gpio, 0);
    }
    return false;
}
#endif

static esp_err_t setup_motor_channel(int idx, mcpwm_timer_handle_t timer) {
    motor_pwm_hw_t *hw = &s_hw[idx];
    const motor_pins_t *p = &s_pins[idx];

    // Configure EN pin first, driven LOW (motor disabled!)
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << p->en_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&en_cfg));
    gpio_set_level(p->en_gpio, 0);

    // Operator in Group 0
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &hw->oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(hw->oper, timer));

    // Comparators update synchronously at timer empty (TEZ) -> glitch-free duty updates!
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags = {
            .update_cmp_on_tez = 1,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(hw->oper, &cmpr_cfg, &hw->cmpr_r));
    ESP_ERROR_CHECK(mcpwm_new_comparator(hw->oper, &cmpr_cfg, &hw->cmpr_l));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hw->cmpr_r, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hw->cmpr_l, 0));

    // Generators for RPWM and LPWM
    mcpwm_generator_config_t gen_r_cfg = { .gen_gpio_num = p->rpwm_gpio };
    mcpwm_generator_config_t gen_l_cfg = { .gen_gpio_num = p->lpwm_gpio };
    ESP_ERROR_CHECK(mcpwm_new_generator(hw->oper, &gen_r_cfg, &hw->gen_r));
    ESP_ERROR_CHECK(mcpwm_new_generator(hw->oper, &gen_l_cfg, &hw->gen_l));

    // Center-aligned symmetric PWM waveform:
    //   On UP crossing comparator   -> HIGH (wait: active-high centered around peak,
    //   or HIGH at TEZ -> LOW on UP compare -> HIGH on DOWN compare).
    //   HIGH on UP compare -> LOW on DOWN compare puts the active pulse symmetrically
    //   centered around PEAK (counter = peak), so cmp = peak - duty_ticks!
    //   Even simpler & zero-glitch:
    //     At TEZ (0): if cmp > 0 we go HIGH; on UP == cmp -> LOW; on DOWN == cmp -> HIGH.
    //     For duty == 0 we force level 0 cleanly so there is ZERO 1-tick runt pulse!
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        hw->gen_r,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, hw->cmpr_r, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, hw->cmpr_r, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        hw->gen_l,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, hw->cmpr_l, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, hw->cmpr_l, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

#if CONFIG_SIMSYNC_ESTOP_GPIO >= 0
    if (s_estop_fault != NULL) {
        mcpwm_brake_config_t brake_cfg = {
            .fault      = s_estop_fault,
            .brake_mode = MCPWM_OPER_BRAKE_MODE_OST,
        };
        ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(hw->oper, &brake_cfg));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
            hw->gen_r,
            MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
            hw->gen_l,
            MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
        if (idx == 0) {
            mcpwm_operator_event_callbacks_t op_cbs = {
                .on_brake_ost = on_estop_brake_cb,
            };
            ESP_ERROR_CHECK(mcpwm_operator_register_event_callbacks(hw->oper, &op_cbs, NULL));
        }
    }
#endif

    // Start forced LOW (100% off)
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(hw->gen_r, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(hw->gen_l, 0, true));
    hw->last_dir = 0;

    return ESP_OK;
}

esp_err_t mcpwm_drive_init(uint16_t pwm12_khz, uint16_t pwm3_khz) {
    uint32_t period_12 = khz_to_period_ticks(pwm12_khz);
    uint32_t period_3  = khz_to_period_ticks(pwm3_khz);
    s_peak_12 = period_12 / 2;
    s_peak_3  = period_3 / 2;

#if CONFIG_SIMSYNC_ESTOP_GPIO >= 0
    mcpwm_gpio_fault_config_t fault_cfg = {
        .group_id = 0,
        .gpio_num = CONFIG_SIMSYNC_ESTOP_GPIO,
        .flags = {
            .active_level = 0,   // Active LOW (NC switch to GND -> opens or pulls low to trip)
            .pull_up      = 1,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&fault_cfg, &s_estop_fault));
    ESP_LOGI(TAG, "Hardware MCPWM E-Stop fault enabled on GPIO%d (active LOW)", CONFIG_SIMSYNC_ESTOP_GPIO);
#endif

    mcpwm_timer_config_t t12_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks  = period_12,
        .flags = {
            .update_period_on_empty = 1,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&t12_cfg, &s_timer_12));

    mcpwm_timer_config_t t3_cfg = t12_cfg;
    t3_cfg.period_ticks = period_3;
    ESP_ERROR_CHECK(mcpwm_new_timer(&t3_cfg, &s_timer_3));

    ESP_ERROR_CHECK(setup_motor_channel(0, s_timer_12));
    ESP_ERROR_CHECK(setup_motor_channel(1, s_timer_12));
    ESP_ERROR_CHECK(setup_motor_channel(2, s_timer_3));

    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer_12));
    ESP_ERROR_CHECK(mcpwm_timer_enable(s_timer_3));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer_12, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timer_3,  MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "MCPWM center-aligned initialized: M1/M2=%ukHz (%lu steps), M3=%ukHz (%lu steps)",
             (unsigned)clamp_khz(pwm12_khz), (unsigned long)s_peak_12,
             (unsigned)clamp_khz(pwm3_khz),  (unsigned long)s_peak_3);
    return ESP_OK;
}

esp_err_t mcpwm_drive_set_freq(uint16_t pwm12_khz, uint16_t pwm3_khz) {
    uint32_t period_12 = khz_to_period_ticks(pwm12_khz);
    uint32_t period_3  = khz_to_period_ticks(pwm3_khz);
    esp_err_t err1 = mcpwm_timer_set_period(s_timer_12, period_12);
    esp_err_t err2 = mcpwm_timer_set_period(s_timer_3,  period_3);
    if (err1 == ESP_OK) s_peak_12 = period_12 / 2;
    if (err2 == ESP_OK) s_peak_3  = period_3 / 2;
    return (err1 != ESP_OK) ? err1 : err2;
}

void mcpwm_drive_set_duty(int motor_idx, int16_t signed_duty, bool invert_dir) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;

    motor_pwm_hw_t *hw = &s_hw[motor_idx];
    int32_t drive = signed_duty;
    if (drive > SIMSYNC_DUTY_MAX)  drive = SIMSYNC_DUTY_MAX;
    if (drive < -SIMSYNC_DUTY_MAX) drive = -SIMSYNC_DUTY_MAX;
    if (invert_dir) drive = -drive;

    uint32_t peak = (motor_idx < 2) ? s_peak_12 : s_peak_3;

    if (drive > 0) {
        uint32_t cmp = ((uint32_t)drive * peak) / SIMSYNC_DUTY_MAX;
        if (cmp == 0) cmp = 1;

        // Always keep LPWM forced LOW first to guarantee zero overlap on reversal
        if (hw->last_dir != 1) {
            mcpwm_generator_set_force_level(hw->gen_l, 0, true);
            mcpwm_comparator_set_compare_value(hw->cmpr_l, 0);
        }
        if (cmp >= peak) {
            mcpwm_generator_set_force_level(hw->gen_r, 1, true);
        } else {
            mcpwm_comparator_set_compare_value(hw->cmpr_r, cmp);
            mcpwm_generator_set_force_level(hw->gen_r, -1, false);
        }
        hw->last_dir = 1;
    } else if (drive < 0) {
        uint32_t cmp = ((uint32_t)(-drive) * peak) / SIMSYNC_DUTY_MAX;
        if (cmp == 0) cmp = 1;

        // Always keep RPWM forced LOW first to guarantee zero overlap on reversal
        if (hw->last_dir != -1) {
            mcpwm_generator_set_force_level(hw->gen_r, 0, true);
            mcpwm_comparator_set_compare_value(hw->cmpr_r, 0);
        }
        if (cmp >= peak) {
            mcpwm_generator_set_force_level(hw->gen_l, 1, true);
        } else {
            mcpwm_comparator_set_compare_value(hw->cmpr_l, cmp);
            mcpwm_generator_set_force_level(hw->gen_l, -1, false);
        }
        hw->last_dir = -1;
    } else {
        if (hw->last_dir != 0) {
            mcpwm_generator_set_force_level(hw->gen_r, 0, true);
            mcpwm_generator_set_force_level(hw->gen_l, 0, true);
            mcpwm_comparator_set_compare_value(hw->cmpr_r, 0);
            mcpwm_comparator_set_compare_value(hw->cmpr_l, 0);
            hw->last_dir = 0;
        }
    }
}

void mcpwm_drive_disable(int motor_idx) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    motor_pwm_hw_t *hw = &s_hw[motor_idx];
    mcpwm_generator_set_force_level(hw->gen_r, 0, true);
    mcpwm_generator_set_force_level(hw->gen_l, 0, true);
    mcpwm_comparator_set_compare_value(hw->cmpr_r, 0);
    mcpwm_comparator_set_compare_value(hw->cmpr_l, 0);
    hw->last_dir = 0;
    gpio_set_level(s_pins[motor_idx].en_gpio, 0);
}

void mcpwm_drive_disable_all(void) {
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        mcpwm_drive_disable(i);
    }
}

bool mcpwm_drive_enable(int motor_idx) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return false;
    if (s_estop_latched) return false;

    motor_pwm_hw_t *hw = &s_hw[motor_idx];
    mcpwm_generator_set_force_level(hw->gen_r, 0, true);
    mcpwm_generator_set_force_level(hw->gen_l, 0, true);
    mcpwm_comparator_set_compare_value(hw->cmpr_r, 0);
    mcpwm_comparator_set_compare_value(hw->cmpr_l, 0);
    hw->last_dir = 0;

    // Toggle EN LOW -> 5 us -> HIGH to reset any latched BTS7960 overtemp/overcurrent fault
    gpio_set_level(s_pins[motor_idx].en_gpio, 0);
    esp_rom_delay_us(5);
    gpio_set_level(s_pins[motor_idx].en_gpio, 1);
    return true;
}

bool mcpwm_drive_is_estop_latched(void) {
    return s_estop_latched;
}

bool mcpwm_drive_clear_estop(void) {
#if CONFIG_SIMSYNC_ESTOP_GPIO >= 0
    if (s_estop_fault != NULL) {
        for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
            esp_err_t err = mcpwm_operator_recover_from_fault(s_hw[i].oper, s_estop_fault);
            if (err != ESP_OK) {
                return false; // Physical E-Stop still active!
            }
        }
    }
#endif
    s_estop_latched = false;
    return true;
}
