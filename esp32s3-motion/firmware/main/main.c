#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "motion_config.h"
#include "nvs_store.h"
#include "adc_engine.h"
#include "mcpwm_drive.h"
#include "pid_control.h"
#include "protocol.h"

static const char *TAG = "simsync_s3";

static TaskHandle_t s_rt_task_handle = NULL;
static volatile int64_t s_isr_fire_us = 0;

// Hardware gptimer ISR running in IRAM: wakes Core 1 RT control task with <2 us jitter
static bool IRAM_ATTR pid_timer_alarm_cb(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_ctx) {
    (void)timer;
    (void)edata;
    (void)user_ctx;
    s_isr_fire_us = esp_timer_get_time();
    BaseType_t hp_woken = pdFALSE;
    if (s_rt_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_rt_task_handle, &hp_woken);
    }
    return hp_woken == pdTRUE;
}

static void rt_control_task(void *arg) {
    (void)arg;

    // Configure dedicated 1 MHz hardware gptimer for exact 2 kHz (500 us) / 4 kHz (250 us) ticks
    const uint64_t period_us = 1000000ULL / (uint64_t)CONFIG_SIMSYNC_PID_FREQ_HZ;
    const float    nom_dt    = 1.0f / (float)CONFIG_SIMSYNC_PID_FREQ_HZ;

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t t_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 us resolution
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&t_cfg, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = pid_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    ESP_LOGI(TAG, "Core 1 RT PID loop active @ %d Hz (%llu us period)",
             CONFIG_SIMSYNC_PID_FREQ_HZ, (unsigned long long)period_us);

    int64_t prev_tick_us = esp_timer_get_time();
    adc_batch_t adc_batch;

    while (1) {
        // Sleep until hardware gptimer ISR wakes us
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us    = esp_timer_get_time();
        int64_t fire_us   = s_isr_fire_us;
        uint16_t jitter   = (now_us >= fire_us) ? (uint16_t)(now_us - fire_us) : 0;
        float dt_sec      = (float)(now_us - prev_tick_us) * 1e-6f;
        prev_tick_us      = now_us;

        // Guard against crazy dt on first tick or debug pause
        if (dt_sec <= 0.0f || dt_sec > nom_dt * 4.0f) {
            dt_sec = nom_dt;
        }

        adc_engine_sample(&adc_batch);
        pid_control_step(&adc_batch, dt_sec, jitter);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, " SimSync ESP32-S3 Ultra-Low-Noise 2DOF/3DOF Motion Firmware");
    ESP_LOGI(TAG, " RF: OFF | ADC1: GDMA %d Hz | MCPWM: Center-Aligned | USB: Native",
             CONFIG_SIMSYNC_ADC_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "============================================================");

    // 1. NVS flash + load saved controller profile (or safe bench defaults)
    ESP_ERROR_CHECK(nvs_store_init());
    controller_cfg_t cfg;
    nvs_store_load(&cfg);

    // 2. Initialize MCPWM FIRST with all motors strictly DISABLED (EN=LOW, PWM=0)
    ESP_ERROR_CHECK(mcpwm_drive_init(cfg.pwm12_khz, cfg.pwm3_khz));

    // 3. Start continuous GDMA ADC1 + eFuse curve-fitting calibration
    ESP_ERROR_CHECK(adc_engine_init());
    adc_batch_t warmup;
    adc_engine_warmup(&warmup, 25);

    // 4. Initialize PID state locked to current pot positions (motors remain DISABLED)
    pid_control_init(&cfg, &warmup);

    // 5. Initialize native USB Serial/JTAG transport on Core 0
    ESP_ERROR_CHECK(protocol_init());

    // 6. Launch deterministic Real-Time Control Loop pinned exclusively to Core 1
    xTaskCreatePinnedToCore(
        rt_control_task,
        "rt_pid_c1",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        &s_rt_task_handle,
        1);

    // 7. Launch USB Serial/JTAG Dual-Protocol & Telemetry Worker pinned to Core 0
    xTaskCreatePinnedToCore(
        protocol_task,
        "usb_proto_c0",
        6144,
        NULL,
        10,
        NULL,
        0);

    ESP_LOGI(TAG, "System ready. All motors DISABLED on boot (safe).");
}
