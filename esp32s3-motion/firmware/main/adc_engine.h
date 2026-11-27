#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "motion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    value_12[SIMSYNC_NUM_MOTORS];      // Oversampled & calibrated 0..4095
    uint16_t p2p_12[SIMSYNC_NUM_MOTORS];        // Peak-to-peak spread within DMA batch
    uint16_t sample_count[SIMSYNC_NUM_MOTORS];  // Number of samples averaged
    bool     overflow;                          // True if DMA pool overflowed
} adc_batch_t;

/**
 * Initialize ESP32-S3 ADC1 continuous GDMA mode + eFuse curve-fitting calibration
 * (and optional hardware IIR filters in 2DOF mode).
 */
esp_err_t adc_engine_init(void);

/**
 * Drain all available GDMA conversion frames non-blockingly and compute
 * outlier-trimmed oversampled 12-bit calibrated positions for all 3 motors.
 * Called from Core 1 real-time PID task on every tick.
 */
void adc_engine_sample(adc_batch_t *out);

/**
 * Blocking warm-up read (for startup center initialization before enabling motors).
 */
void adc_engine_warmup(adc_batch_t *out, uint32_t settle_ms);

#ifdef __cplusplus
}
#endif
