#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "motion_config.h"
#include "adc_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Populate controller_cfg_t with safe bench defaults matching SimSync / SMC3.
 */
void pid_default_config(controller_cfg_t *cfg);

/**
 * Initialize PID runtime state from loaded config and initial ADC warmup batch.
 * All motors start strictly DISABLED with target == current feedback.
 */
void pid_control_init(const controller_cfg_t *cfg, const adc_batch_t *warmup);

/**
 * Execute one deterministic control tick on Core 1.
 * Updates feedback, safety cutoffs, jog/PID math, MCPWM output, and telemetry snapshot.
 */
void pid_control_step(const adc_batch_t *adc, float dt_sec, uint16_t jitter_us);

/**
 * Thread-safe snapshot for Core 0 telemetry/readback (never blocks Core 1).
 */
void pid_get_snapshot(telem_snapshot_t *out);

/**
 * Thread-safe copy of active controller config (for NVS save or host readback).
 */
void pid_get_config(controller_cfg_t *out);

// ---- Thread-safe command setters called from Core 0 protocol task ----

void pid_touch_comms(void);
void pid_set_target_12(int motor_idx, uint16_t target_12);
void pid_set_targets_batch_12(const uint16_t targets_12[SIMSYNC_NUM_MOTORS], uint8_t mask);

void pid_set_gains(int motor_idx, float kp, float ki, float kd, float kff, uint16_t ks,
                   float lpf_hz, float hpf_hz);
void pid_set_kp(int motor_idx, float kp);
void pid_set_ki(int motor_idx, float ki);
void pid_set_kd(int motor_idx, float kd);
void pid_set_ks(int motor_idx, uint16_t ks);
bool pid_set_lpf(int motor_idx, float lpf_hz);
bool pid_set_hpf(int motor_idx, float hpf_hz);

void pid_set_pwm_limits(int motor_idx, uint16_t pwm_min, uint16_t pwm_max);
void pid_set_travel_limits_12(int motor_idx, uint16_t cutoff_min_12, uint16_t cutoff_max_12,
                              uint16_t clip_min_12, uint16_t clip_max_12);
void pid_set_deadzone_rev(int motor_idx, uint16_t deadzone_12, uint16_t pwm_rev);
void pid_set_polarity(int motor_idx, bool invert_pot, bool invert_dir);

void pid_set_divider(uint8_t divider);
void pid_set_pwm_khz(uint16_t pwm12_khz, uint16_t pwm3_khz);
void pid_set_active_dof(uint8_t dof);

void pid_enable_motor(int motor_idx);
void pid_enable_mask(uint8_t mask);
void pid_disable_all(void);
void pid_hold_mask(uint8_t mask);

void pid_set_jog_min_12(int motor_idx, uint16_t min_12);
void pid_set_jog_max_12(int motor_idx, uint16_t max_12);
void pid_start_jog(int motor_idx, int16_t duty_1000, uint16_t duration_ms);

bool pid_clear_estop(void);
void pid_reset_timing_stats(void);

#ifdef __cplusplus
}
#endif
