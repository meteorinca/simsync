#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "motion_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize ESP32-S3 MCPWM Group 0 for 3x IBT-2 / BTS7960 H-bridges:
 *   - True dual-PWM (RPWM + LPWM + EN per motor)
 *   - Center-aligned (UP-DOWN) ultrasonic PWM (default 20 kHz, 1000-step resolution)
 *   - Optional hardware GPIO fault input for nanosecond E-Stop shutdown
 * All motors start strictly DISABLED (EN LOW, RPWM=0, LPWM=0).
 */
esp_err_t mcpwm_drive_init(uint16_t pwm12_khz, uint16_t pwm3_khz);

/**
 * Update PWM frequencies at runtime (1..25 kHz, BTS7960 rated max 25 kHz).
 * Motors should be disabled or zero-duty when changing frequency.
 */
esp_err_t mcpwm_drive_set_freq(uint16_t pwm12_khz, uint16_t pwm3_khz);

/**
 * Drive motor channel (0..2) with signed duty (-1000 .. +1000).
 * Positive -> RPWM = duty, LPWM = 0. Negative -> LPWM = |duty|, RPWM = 0.
 * Zero -> both PWM = 0 (active brake while EN HIGH).
 * No unnecessary EN blanking on same-direction ticks.
 */
void mcpwm_drive_set_duty(int motor_idx, int16_t signed_duty, bool invert_dir);

/**
 * Enable motor H-bridge: sets PWM=0, toggles EN LOW->HIGH to clear any latched
 * BTS7960 thermal/overcurrent fault. Fails if hardware E-Stop is active.
 */
bool mcpwm_drive_enable(int motor_idx);

/**
 * Disable motor immediately: forces RPWM=0, LPWM=0, EN=LOW.
 */
void mcpwm_drive_disable(int motor_idx);

/**
 * Disable all 3 motors immediately.
 */
void mcpwm_drive_disable_all(void);

/**
 * Returns true if the hardware E-Stop GPIO fault is currently tripped/latched.
 */
bool mcpwm_drive_is_estop_latched(void);

/**
 * Attempt to clear hardware E-Stop latch (succeeds only if E-Stop button is released).
 */
bool mcpwm_drive_clear_estop(void);

#ifdef __cplusplus
}
#endif
