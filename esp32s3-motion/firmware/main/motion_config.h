#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIMSYNC_NUM_MOTORS          3
#define SIMSYNC_ADC_MAX_12BIT       4095
#define SIMSYNC_ADC_CENTER_12BIT    2048
#define SIMSYNC_DUTY_MAX            1000    // Signed duty: -1000 .. +1000 (0.1% steps)

// SMC3 compatibility constants
#define SMC3_FIRMWARE_VERSION       70      // v0.70 for SMC3Utils / SimSync
#define SMC3_CAPABILITY_FLAGS       7       // bit0: jog, bit1: hold, bit2: filters

// Fault bitmask flags (reported in Turbo v2 telemetry)
#define FAULT_NONE                  0x00
#define FAULT_HW_ESTOP              (1U << 0)
#define FAULT_CUTOFF_M1             (1U << 1)
#define FAULT_CUTOFF_M2             (1U << 2)
#define FAULT_CUTOFF_M3             (1U << 3)
#define FAULT_COMMS_TIMEOUT         (1U << 4)
#define FAULT_ADC_OVERFLOW          (1U << 5)

// 2nd-order Biquad IIR filter (Direct Form II Transposed, float32)
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
    float cutoff_hz;
    float sample_hz;
    bool  active;
} biquad_t;

// Open-loop timed jog state (SimSync compatible, 12-bit bounds)
typedef struct {
    int16_t  duty;          // -1000..1000
    uint16_t min_12;        // 0..4095
    uint16_t max_12;        // 0..4095
    int64_t  deadline_us;   // esp_timer_get_time() deadline
    bool     running;
} jog_state_t;

// Persistent per-motor configuration (stored in NVS)
typedef struct {
    // PID + Feedforward (SMC3-compatible scale: kp_x100 = 2000 means Kp=20.0)
    float    kp;
    float    ki;
    float    kd;
    float    kff;           // Velocity feedforward (Turbo v2)
    uint16_t ks;            // Derivative sample divider (1..100)

    // Filter cutoffs in Hz (0 = off)
    float    lpf_hz;        // Feedback 2nd-order Butterworth LPF (0..250 Hz)
    float    hpf_hz;        // Derivative washout HPF (0..50 Hz)

    // PWM shaping (0..1000 internal scale; SMC3 0..255 maps via *1000/255)
    uint16_t pwm_min;       // Minimum drive offset (0..1000)
    uint16_t pwm_max;       // Drive ceiling (0..1000)
    uint16_t pwm_rev;       // Reverse braking pulse outside clip zone (0..1000)
    uint16_t deadzone_12;   // 12-bit deadzone counts (0..200)

    // Travel safety bounds (12-bit counts: 0..4095)
    uint16_t cutoff_min_12;
    uint16_t cutoff_max_12;
    uint16_t clip_min_12;
    uint16_t clip_max_12;

    // Polarity flags
    uint8_t  invert_pot;
    uint8_t  invert_dir;
} motor_cfg_t;

// Global controller configuration (stored in NVS)
typedef struct {
    uint32_t    magic;
    uint16_t    version;
    uint8_t     active_dof;       // 2 = 2DOF (M1+M2), 3 = 3DOF (M1+M2+M3)
    uint8_t     pid_divider;      // 1..10 (SMC3 [Z] divider)
    uint16_t    pwm12_khz;        // 1..25 kHz (M1/M2 MCPWM timer)
    uint16_t    pwm3_khz;         // 1..25 kHz (M3 MCPWM timer)
    motor_cfg_t motors[SIMSYNC_NUM_MOTORS];
} controller_cfg_t;

// Per-motor live runtime state (owned by Core 1 RT loop)
typedef struct {
    float       feedback_raw_12;    // Oversampled calibrated 12-bit (0..4095)
    float       feedback_filt_12;   // Post-biquad feedback (0..4095)
    float       target_12;          // Commanded target (0..4095)
    float       last_target_12;     // Previous tick target (for feedforward)
    float       target_vel_filt;    // Filtered target velocity (counts/sec)
    float       integrator;         // Anti-windup integral accumulator
    float       last_pos;           // Last derivative sample position
    float       d_term;             // Cached derivative output
    float       d_prev_delta;       // Derivative HPF prev input
    float       d_hp_out;           // Derivative HPF output
    float       d_accum_dt;         // Accumulated dt for Ks > 1
    uint16_t    kd_counter;

    int16_t     duty_out;           // Signed MCPWM duty (-1000..+1000)
    bool        disabled;           // True = EN LOW, PWM 0

    // Live noise diagnostics (updated every tick from DMA batch)
    uint16_t    adc_p2p_12;         // Peak-to-peak spread within tick (12-bit counts)
    float       adc_rms_12;         // Smoothed RMS noise estimate (12-bit counts)
    float       adc_ema_mean;       // Mean tracker for RMS calculation
    uint16_t    adc_samples_tick;   // DMA samples averaged this tick

    biquad_t    lpf;
    jog_state_t jog;
} motor_rt_t;

// Lock-free telemetry snapshot (copied atomically under brief spinlock for Core 0)
typedef struct {
    uint16_t actual_12[SIMSYNC_NUM_MOTORS];
    uint16_t target_12[SIMSYNC_NUM_MOTORS];
    int16_t  duty[SIMSYNC_NUM_MOTORS];
    uint16_t noise_p2p_12[SIMSYNC_NUM_MOTORS];
    uint16_t noise_rms_x100[SIMSYNC_NUM_MOTORS];
    uint16_t adc_samples[SIMSYNC_NUM_MOTORS];
    uint8_t  disabled_mask;         // bit0=M1, bit1=M2, bit2=M3
    uint8_t  fault_flags;
    uint8_t  active_dof;
    uint8_t  pid_divider;
    uint16_t loop_exec_us;          // Last Core 1 loop body runtime (us)
    uint16_t loop_max_us;           // Peak Core 1 loop runtime (us)
    uint16_t loop_jitter_us;        // Peak timer wakeup jitter (us)
    uint32_t loop_count;
} telem_snapshot_t;

// ============================================================================
// Native ESP32-S3 Turbo v2 Binary Protocol (0xAA 0x55 <type> <len> <data> <crc8>)
// ============================================================================
#define TURBO_SYNC_0                0xAA
#define TURBO_SYNC_1                0x55

// Host -> ESP32-S3 commands
#define TURBO_CMD_TARGETS_12        0x01  // 3x uint16 target (0..4095) + uint8 mask (7 bytes)
#define TURBO_CMD_SET_GAINS         0x02  // uint8 motor + 6x float32 (kp,ki,kd,kff,lpf,hpf) + uint16 ks (27 bytes)
#define TURBO_CMD_SET_LIMITS        0x03  // uint8 motor + 8x uint16 + 2x uint8 (19 bytes)
#define TURBO_CMD_CONTROL           0x04  // uint8 op + uint16 arg1 + uint16 arg2 (5 bytes)
#define TURBO_CMD_GET_CONFIG        0x05  // uint8 motor (0..2, or 0xFF for global) (1 byte)

// TURBO_CMD_CONTROL ops
#define TURBO_OP_STOP_ALL           0x00
#define TURBO_OP_ENABLE_MASK        0x01  // arg1 = bitmask (1..7)
#define TURBO_OP_HOLD_MASK          0x02  // arg1 = bitmask (1..7)
#define TURBO_OP_CLEAR_ESTOP        0x03
#define TURBO_OP_SAVE_NVS           0x04
#define TURBO_OP_SET_STREAM_HZ      0x05  // arg1 = 0..250 Hz
#define TURBO_OP_SET_PWM_KHZ        0x06  // arg1 = pwm12_khz (1..25), arg2 = pwm3_khz (1..25)
#define TURBO_OP_SET_ACTIVE_DOF     0x07  // arg1 = 2 or 3
#define TURBO_OP_RESET_STATS        0x08  // reset peak loop/jitter stats

// ESP32-S3 -> Host packets
#define TURBO_PKT_TELEMETRY         0x81  // Packed telemetry frame
#define TURBO_PKT_MOTOR_CFG         0x82  // Response to GET_CONFIG(0..2)
#define TURBO_PKT_GLOBAL_CFG        0x83  // Response to GET_CONFIG(0xFF)
#define TURBO_PKT_ACK               0x84  // { cmd_type, status }

#ifdef __cplusplus
}
#endif
