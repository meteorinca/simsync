#include "pid_control.h"
#include "mcpwm_drive.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_timer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Comms watchdog: drop drive authority after 15 seconds of silence (matching SMC3)
#define COMMS_TIMEOUT_US    (15000000LL)

static portMUX_TYPE      s_lock = portMUX_INITIALIZER_UNLOCKED;
static controller_cfg_t  s_cfg;
static motor_rt_t        s_rt[SIMSYNC_NUM_MOTORS];
static telem_snapshot_t  s_snap;

static uint8_t           s_div_counter = 0;
static float             s_div_accum_dt = 0.0f;
static uint8_t           s_fault_flags = FAULT_NONE;
static int64_t           s_last_comms_us = 0;
static uint16_t          s_loop_max_us = 0;
static uint16_t          s_jitter_max_us = 0;
static uint32_t          s_loop_count = 0;

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) {
    if (v < (int32_t)lo) return lo;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

// ============================================================================
// 2nd-order Butterworth Biquad LPF (Q = 0.70710678)
// Gives 40 dB/decade noise rejection with maximally flat passband!
// ============================================================================
static void biquad_configure_lpf(biquad_t *bq, float cutoff_hz, float sample_hz, float init_val) {
    if (cutoff_hz <= 0.0f || sample_hz <= 0.0f || cutoff_hz >= sample_hz * 0.45f) {
        bq->active    = false;
        bq->cutoff_hz = cutoff_hz;
        bq->sample_hz = sample_hz;
        bq->z1        = 0.0f;
        bq->z2        = 0.0f;
        return;
    }

    float w0    = 2.0f * (float)M_PI * cutoff_hz / sample_hz;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / 1.41421356f; // Q = 1/sqrt(2) Butterworth

    float a0 = 1.0f + alpha;
    bq->b0 = ((1.0f - cosw0) * 0.5f) / a0;
    bq->b1 = (1.0f - cosw0) / a0;
    bq->b2 = bq->b0;
    bq->a1 = (-2.0f * cosw0) / a0;
    bq->a2 = (1.0f - alpha) / a0;

    // Pre-charge Direct Form II Transposed state for zero startup transient
    bq->z1 = init_val * (1.0f - bq->b0);
    bq->z2 = init_val * (bq->b2 - bq->a2);
    bq->cutoff_hz = cutoff_hz;
    bq->sample_hz = sample_hz;
    bq->active    = true;
}

static inline float biquad_process(biquad_t *bq, float x) {
    if (!bq->active) return x;
    float y = bq->b0 * x + bq->z1;
    bq->z1  = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2  = bq->b2 * x - bq->a2 * y;
    return y;
}

void pid_default_config(controller_cfg_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic       = 0x53533353UL; // "SS3S"
    cfg->version     = 1;
    cfg->active_dof  = CONFIG_SIMSYNC_ACTIVE_DOF;
    cfg->pid_divider = 1;
    cfg->pwm12_khz   = CONFIG_SIMSYNC_PWM_FREQ_HZ / 1000;
    cfg->pwm3_khz    = CONFIG_SIMSYNC_PWM_FREQ_HZ / 1000;

    const bool inv_pot[3] = {
#ifdef CONFIG_SIMSYNC_M1_INVERT_POT
        true,
#else
        false,
#endif
#ifdef CONFIG_SIMSYNC_M2_INVERT_POT
        true,
#else
        false,
#endif
#ifdef CONFIG_SIMSYNC_M3_INVERT_POT
        true,
#else
        false,
#endif
    };

    const bool inv_dir[3] = {
#ifdef CONFIG_SIMSYNC_M1_INVERT_DIR
        true,
#else
        false,
#endif
#ifdef CONFIG_SIMSYNC_M2_INVERT_DIR
        true,
#else
        false,
#endif
#ifdef CONFIG_SIMSYNC_M3_INVERT_DIR
        true,
#else
        false,
#endif
    };

    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        motor_cfg_t *m = &cfg->motors[i];
        m->kp            = 20.0f;    // Matches SimSync safe bench default (P20 / D0.4)
        m->ki            = 0.0f;
        m->kd            = 0.40f;
        m->kff           = 0.0f;
        m->ks            = 1;
        m->lpf_hz        = 80.0f;    // Clean 80 Hz 2nd-order Butterworth feedback LPF
        m->hpf_hz        = 0.0f;
        m->pwm_min       = 0;        // 0..1000 scale
        m->pwm_max       = 392;      // ~100/255 safe starting ceiling
        m->pwm_rev       = 196;      // ~50/255 reverse brake
        m->deadzone_12   = 4;        // 1 count in 10-bit = 4 counts in 12-bit
        m->cutoff_min_12 = 92;       // 23 * 4
        m->cutoff_max_12 = 4095 - 92;
        m->clip_min_12   = 160;      // 40 * 4
        m->clip_max_12   = 4095 - 160;
        m->invert_pot    = inv_pot[i] ? 1 : 0;
        m->invert_dir    = inv_dir[i] ? 1 : 0;
    }
}

static inline float apply_pot_invert(int idx, float val_12) {
    return s_cfg.motors[idx].invert_pot ? (4095.0f - val_12) : val_12;
}

static void disable_motor_locked(int i) {
    s_rt[i].disabled   = true;
    s_rt[i].duty_out   = 0;
    s_rt[i].integrator = 0.0f;
    s_rt[i].d_term     = 0.0f;
    s_rt[i].jog.running = false;
    mcpwm_drive_disable(i);
}

void pid_control_init(const controller_cfg_t *cfg, const adc_batch_t *warmup) {
    portENTER_CRITICAL(&s_lock);
    s_cfg = *cfg;
    memset(s_rt, 0, sizeof(s_rt));
    memset(&s_snap, 0, sizeof(s_snap));

    float sample_hz = (float)CONFIG_SIMSYNC_PID_FREQ_HZ / (float)(s_cfg.pid_divider > 0 ? s_cfg.pid_divider : 1);

    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        float pos = apply_pot_invert(i, warmup->value_12[i]);
        s_rt[i].feedback_raw_12  = pos;
        s_rt[i].feedback_filt_12 = pos;
        s_rt[i].target_12        = pos;
        s_rt[i].last_target_12   = pos;
        s_rt[i].last_pos         = pos;
        s_rt[i].adc_ema_mean     = pos;
        s_rt[i].disabled         = true;
        s_rt[i].jog.min_12       = SIMSYNC_ADC_CENTER_12BIT;
        s_rt[i].jog.max_12       = SIMSYNC_ADC_CENTER_12BIT;
        biquad_configure_lpf(&s_rt[i].lpf, s_cfg.motors[i].lpf_hz, sample_hz, pos);
        mcpwm_drive_disable(i);
    }

    s_last_comms_us = esp_timer_get_time();
    s_fault_flags   = FAULT_NONE;
    s_loop_count    = 0;
    s_loop_max_us   = 0;
    s_jitter_max_us = 0;
    portEXIT_CRITICAL(&s_lock);
}

static void end_jog_locked(int i) {
    s_rt[i].jog.running = false;
    s_rt[i].target_12   = s_rt[i].feedback_raw_12;
    disable_motor_locked(i);
}

void pid_control_step(const adc_batch_t *adc, float dt_sec, uint16_t jitter_us) {
    int64_t t_start_us = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);

    if (jitter_us > s_jitter_max_us) {
        s_jitter_max_us = jitter_us;
    }

    // Check hardware E-Stop latch
    if (mcpwm_drive_is_estop_latched()) {
        s_fault_flags |= FAULT_HW_ESTOP;
        for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
            disable_motor_locked(i);
        }
    } else {
        s_fault_flags &= ~FAULT_HW_ESTOP;
    }

    if (adc->overflow) {
        s_fault_flags |= FAULT_ADC_OVERFLOW;
    }

    // Comms watchdog authority scaling (1.0 normal, 0.25 after 15s silence)
    float power_mult = 1.0f;
    if ((t_start_us - s_last_comms_us) > COMMS_TIMEOUT_US) {
        power_mult = 0.25f;
        s_fault_flags |= FAULT_COMMS_TIMEOUT;
    } else {
        s_fault_flags &= ~FAULT_COMMS_TIMEOUT;
    }

    // Always update live raw feedback & noise diagnostics every base tick
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        float pos = apply_pot_invert(i, adc->value_12[i]);
        s_rt[i].feedback_raw_12  = pos;
        s_rt[i].adc_p2p_12       = adc->p2p_12[i];
        s_rt[i].adc_samples_tick = adc->sample_count[i];

        // Exponential moving RMS noise tracker (tracks micro-jitter while stationary)
        float diff = pos - s_rt[i].adc_ema_mean;
        s_rt[i].adc_ema_mean += 0.05f * diff;
        float sq = diff * diff;
        float prev_var = s_rt[i].adc_rms_12 * s_rt[i].adc_rms_12;
        float new_var  = 0.98f * prev_var + 0.02f * sq;
        s_rt[i].adc_rms_12 = sqrtf(new_var > 0.0f ? new_var : 0.0f);
    }

    s_div_counter++;
    s_div_accum_dt += dt_sec;
    uint8_t divider = (s_cfg.pid_divider >= 1 && s_cfg.pid_divider <= 10) ? s_cfg.pid_divider : 1;

    if (s_div_counter >= divider) {
        float pid_dt = s_div_accum_dt;
        if (pid_dt < 1e-6f) pid_dt = dt_sec;
        float pid_hz = 1.0f / pid_dt;
        s_div_counter  = 0;
        s_div_accum_dt = 0.0f;

        for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
            const motor_cfg_t *mc = &s_cfg.motors[i];
            motor_rt_t *rt        = &s_rt[i];

            float raw_pos = rt->feedback_raw_12;

            // Reconfigure biquad if cutoff or effective sample rate shifted noticeably
            if (fabsf(rt->lpf.cutoff_hz - mc->lpf_hz) > 0.05f ||
                (mc->lpf_hz > 0.0f && fabsf(rt->lpf.sample_hz - pid_hz) > pid_hz * 0.05f)) {
                biquad_configure_lpf(&rt->lpf, mc->lpf_hz, pid_hz, raw_pos);
            }

            // Hard travel cutoff safety ALWAYS uses raw unfiltered feedback!
            if (raw_pos < (float)mc->cutoff_min_12 || raw_pos > (float)mc->cutoff_max_12) {
                s_fault_flags |= (uint8_t)(FAULT_CUTOFF_M1 << i);
                if (rt->jog.running) {
                    end_jog_locked(i);
                } else {
                    disable_motor_locked(i);
                }
                rt->feedback_filt_12 = raw_pos;
                continue;
            } else {
                s_fault_flags &= ~(uint8_t)(FAULT_CUTOFF_M1 << i);
            }

            // Open-loop timed jog mode (SimSync compatible)
            if (rt->jog.running) {
                if (rt->disabled ||
                    raw_pos <= (float)rt->jog.min_12 ||
                    raw_pos >= (float)rt->jog.max_12 ||
                    t_start_us >= rt->jog.deadline_us) {
                    end_jog_locked(i);
                } else {
                    rt->duty_out = rt->jog.duty;
                    mcpwm_drive_set_duty(i, rt->duty_out, mc->invert_dir != 0);
                }
                rt->feedback_filt_12 = raw_pos;
                continue;
            }

            if (rt->disabled) {
                rt->feedback_filt_12 = raw_pos;
                rt->integrator       = 0.0f;
                rt->last_pos         = raw_pos;
                rt->last_target_12   = rt->target_12;
                rt->target_vel_filt  = 0.0f;
                rt->d_term           = 0.0f;
                rt->d_prev_delta     = 0.0f;
                rt->d_hp_out         = 0.0f;
                rt->kd_counter       = 0;
                rt->d_accum_dt       = 0.0f;
                rt->duty_out         = 0;
                // Keep biquad pre-charged to raw_pos while disabled for bumpless enable
                if (rt->lpf.active) {
                    rt->lpf.z1 = raw_pos * (1.0f - rt->lpf.b0);
                    rt->lpf.z2 = raw_pos * (rt->lpf.b2 - rt->lpf.a2);
                }
                continue;
            }

            // 1. 2nd-order Butterworth LPF on feedback
            float pos = biquad_process(&rt->lpf, raw_pos);
            rt->feedback_filt_12 = pos;

            // 2. Position error (converted to 10-bit equivalent units so SMC3 Kp/Ki/Kd gains
            //    produce the EXACT expected stiffness & feel without user re-tuning!)
            //    err_10 = (target_12 - pos_12) * 0.25f -> preserves full 12-bit fractional precision!
            float err_12 = rt->target_12 - pos;
            float err_10 = err_12 * 0.25f;

            // 3. Conditional integral with anti-windup clamp
            if (fabsf(err_12) <= (float)mc->deadzone_12) {
                rt->integrator = 0.0f;
            } else {
                // Scale by pid_dt / 0.00025f so Ki feels consistent across 2 kHz / 4 kHz loop rates
                float dt_norm = pid_dt * 4000.0f;
                rt->integrator = clampf(rt->integrator + err_10 * dt_norm, -1024.0f, 1024.0f);
            }

            // 4. Derivative-on-measurement (zero derivative kick on setpoint steps!)
            rt->kd_counter++;
            rt->d_accum_dt += pid_dt;
            uint16_t ks = (mc->ks >= 1) ? mc->ks : 1;
            if (rt->kd_counter >= ks) {
                float delta_10 = (pos - rt->last_pos) * 0.25f;
                rt->last_pos   = pos;

                // Optional derivative HPF washout (matching SimSync [$/%/&] feature)
                float d_in = delta_10;
                if (mc->hpf_hz > 0.05f) {
                    float alpha = 1.0f / (1.0f + 2.0f * (float)M_PI * mc->hpf_hz * rt->d_accum_dt);
                    rt->d_hp_out     = alpha * (rt->d_hp_out + delta_10 - rt->d_prev_delta);
                    rt->d_prev_delta = delta_10;
                    d_in             = rt->d_hp_out;
                } else {
                    rt->d_prev_delta = delta_10;
                    rt->d_hp_out     = delta_10;
                }

                // Normalize derivative rate to 4 kHz reference tick so Kd matches SMC3 scale
                float rate_scale = (rt->d_accum_dt > 1e-6f) ? (0.00025f * (float)ks / rt->d_accum_dt) : 1.0f;
                rt->d_term       = mc->kd * d_in * rate_scale;
                rt->kd_counter   = 0;
                rt->d_accum_dt   = 0.0f;
            }

            // 5. Velocity feedforward on smoothed setpoint derivative (Turbo v2 enhancement)
            float ff_term = 0.0f;
            if (mc->kff > 0.0001f) {
                float raw_vel = ((rt->target_12 - rt->last_target_12) * 0.25f) / pid_dt;
                rt->target_vel_filt += 0.15f * (raw_vel - rt->target_vel_filt);
                ff_term = mc->kff * rt->target_vel_filt * 0.01f;
            }
            rt->last_target_12 = rt->target_12;

            // 6. Combine P + I - D + FF -> SMC3 scaled output (100.0/128.0 -> 0..255 -> 0..1000)
            //    In SMC3: ((kp*100 * err_10) >> 7) gives PWM in 0..255.
            //    Multiply by (1000 / 255) = 3.9215686 -> exact 0..1000 high-res MCPWM duty!
            float smc3_pwm = ((mc->kp * err_10 + mc->ki * rt->integrator - rt->d_term) * (100.0f / 128.0f) + ff_term) * power_mult;
            float duty_f   = clampf(smc3_pwm * (1000.0f / 255.0f), -1000.0f, 1000.0f);

            // 7. Apply reverse braking (outside input clip), deadzone, PWMmin/PWMmax shaping
            int16_t drive = 0;
            if (raw_pos > (float)mc->clip_max_12 && mc->pwm_rev != 0) {
                drive = -(int16_t)mc->pwm_rev;
            } else if (raw_pos < (float)mc->clip_min_12 && mc->pwm_rev != 0) {
                drive = (int16_t)mc->pwm_rev;
            } else if (fabsf(rt->target_12 - raw_pos) <= (float)mc->deadzone_12) {
                drive = 0;
            } else {
                float mag = fabsf(duty_f) + (float)mc->pwm_min;
                if (mag > (float)mc->pwm_max) mag = (float)mc->pwm_max;
                if (mag > (float)SIMSYNC_DUTY_MAX) mag = (float)SIMSYNC_DUTY_MAX;
                int16_t mag_i = (int16_t)lroundf(mag);
                drive = (duty_f >= 0.0f) ? mag_i : -mag_i;
            }

            rt->duty_out = drive;
            mcpwm_drive_set_duty(i, drive, mc->invert_dir != 0);
        }

        s_loop_count++;
    }

    int64_t t_end_us = esp_timer_get_time();
    uint16_t exec_us = (uint16_t)((t_end_us >= t_start_us) ? (t_end_us - t_start_us) : 0);
    if (exec_us > s_loop_max_us) {
        s_loop_max_us = exec_us;
    }

    // Update telemetry snapshot atomically
    uint8_t dmask = 0;
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        s_snap.actual_12[i]      = clamp_u16((int32_t)lroundf(s_rt[i].feedback_raw_12), 0, 4095);
        s_snap.target_12[i]      = clamp_u16((int32_t)lroundf(s_rt[i].target_12), 0, 4095);
        s_snap.duty[i]           = s_rt[i].duty_out;
        s_snap.noise_p2p_12[i]   = s_rt[i].adc_p2p_12;
        s_snap.noise_rms_x100[i] = clamp_u16((int32_t)lroundf(s_rt[i].adc_rms_12 * 100.0f), 0, 65535);
        s_snap.adc_samples[i]    = s_rt[i].adc_samples_tick;
        if (s_rt[i].disabled) dmask |= (uint8_t)(1U << i);
    }
    s_snap.disabled_mask  = dmask;
    s_snap.fault_flags    = s_fault_flags;
    s_snap.active_dof     = s_cfg.active_dof;
    s_snap.pid_divider    = s_cfg.pid_divider;
    s_snap.loop_exec_us   = exec_us;
    s_snap.loop_max_us    = s_loop_max_us;
    s_snap.loop_jitter_us = s_jitter_max_us;
    s_snap.loop_count     = s_loop_count;

    portEXIT_CRITICAL(&s_lock);
}

void pid_get_snapshot(telem_snapshot_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = s_snap;
    portEXIT_CRITICAL(&s_lock);
}

void pid_get_config(controller_cfg_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = s_cfg;
    portEXIT_CRITICAL(&s_lock);
}

void pid_touch_comms(void) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    s_last_comms_us = now;
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_target_12(int motor_idx, uint16_t target_12) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    const motor_cfg_t *mc = &s_cfg.motors[motor_idx];
    uint16_t clamped = clamp_u16(target_12, mc->clip_min_12, mc->clip_max_12);
    s_rt[motor_idx].target_12 = (float)clamped;
    s_last_comms_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_targets_batch_12(const uint16_t targets_12[SIMSYNC_NUM_MOTORS], uint8_t mask) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (mask & (1U << i)) {
            const motor_cfg_t *mc = &s_cfg.motors[i];
            uint16_t clamped = clamp_u16(targets_12[i], mc->clip_min_12, mc->clip_max_12);
            s_rt[i].target_12 = (float)clamped;
        }
    }
    s_last_comms_us = now;
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_gains(int motor_idx, float kp, float ki, float kd, float kff, uint16_t ks,
                   float lpf_hz, float hpf_hz) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    motor_cfg_t *mc = &s_cfg.motors[motor_idx];
    mc->kp     = clampf(kp,  0.0f, 327.67f);
    mc->ki     = clampf(ki,  0.0f, 327.67f);
    mc->kd     = clampf(kd,  0.0f, 327.67f);
    mc->kff    = clampf(kff, 0.0f, 100.0f);
    mc->ks     = clamp_u16(ks, 1, 100);
    mc->lpf_hz = clampf(lpf_hz, 0.0f, 250.0f);
    mc->hpf_hz = clampf(hpf_hz, 0.0f, 50.0f);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_kp(int motor_idx, float kp) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.motors[motor_idx].kp = clampf(kp, 0.0f, 327.67f);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_ki(int motor_idx, float ki) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.motors[motor_idx].ki = clampf(ki, 0.0f, 327.67f);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_kd(int motor_idx, float kd) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.motors[motor_idx].kd = clampf(kd, 0.0f, 327.67f);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_ks(int motor_idx, uint16_t ks) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.motors[motor_idx].ks = clamp_u16(ks, 1, 100);
    portEXIT_CRITICAL(&s_lock);
}

bool pid_set_lpf(int motor_idx, float lpf_hz) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return false;
    if (lpf_hz < 0.0f || lpf_hz > 250.0f) return false;
    bool ok = false;
    portENTER_CRITICAL(&s_lock);
    if (s_rt[motor_idx].disabled && !s_rt[motor_idx].jog.running) {
        s_cfg.motors[motor_idx].lpf_hz = lpf_hz;
        ok = true;
    }
    portEXIT_CRITICAL(&s_lock);
    return ok;
}

bool pid_set_hpf(int motor_idx, float hpf_hz) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return false;
    if (hpf_hz < 0.0f || hpf_hz > 50.0f) return false;
    bool ok = false;
    portENTER_CRITICAL(&s_lock);
    if (s_rt[motor_idx].disabled && !s_rt[motor_idx].jog.running) {
        s_cfg.motors[motor_idx].hpf_hz = hpf_hz;
        s_rt[motor_idx].d_prev_delta   = 0.0f;
        s_rt[motor_idx].d_hp_out       = 0.0f;
        ok = true;
    }
    portEXIT_CRITICAL(&s_lock);
    return ok;
}

void pid_set_pwm_limits(int motor_idx, uint16_t pwm_min, uint16_t pwm_max) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.motors[motor_idx].pwm_min = clamp_u16(pwm_min, 0, SIMSYNC_DUTY_MAX);
    s_cfg.motors[motor_idx].pwm_max = clamp_u16(pwm_max, 0, SIMSYNC_DUTY_MAX);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_travel_limits_12(int motor_idx, uint16_t cutoff_min_12, uint16_t cutoff_max_12,
                              uint16_t clip_min_12, uint16_t clip_max_12) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    if (cutoff_min_12 >= cutoff_max_12 || clip_min_12 >= clip_max_12) return;
    portENTER_CRITICAL(&s_lock);
    motor_cfg_t *mc   = &s_cfg.motors[motor_idx];
    mc->cutoff_min_12 = clamp_u16(cutoff_min_12, 0, 2000);
    mc->cutoff_max_12 = clamp_u16(cutoff_max_12, 2095, 4095);
    mc->clip_min_12   = clamp_u16(clip_min_12, mc->cutoff_min_12, 2000);
    mc->clip_max_12   = clamp_u16(clip_max_12, 2095, mc->cutoff_max_12);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_deadzone_rev(int motor_idx, uint16_t deadzone_12, uint16_t pwm_rev) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.motors[motor_idx].deadzone_12 = clamp_u16(deadzone_12, 0, 200);
    s_cfg.motors[motor_idx].pwm_rev     = clamp_u16(pwm_rev, 0, SIMSYNC_DUTY_MAX);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_polarity(int motor_idx, bool invert_pot, bool invert_dir) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    if (s_rt[motor_idx].disabled) {
        s_cfg.motors[motor_idx].invert_pot = invert_pot ? 1 : 0;
        s_cfg.motors[motor_idx].invert_dir = invert_dir ? 1 : 0;
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_divider(uint8_t divider) {
    portENTER_CRITICAL(&s_lock);
    s_cfg.pid_divider = (uint8_t)clamp_u16(divider, 1, 10);
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_pwm_khz(uint16_t pwm12_khz, uint16_t pwm3_khz) {
    uint16_t f12 = clamp_u16(pwm12_khz, 1, 25);
    uint16_t f3  = clamp_u16(pwm3_khz,  1, 25);
    portENTER_CRITICAL(&s_lock);
    s_cfg.pwm12_khz = f12;
    s_cfg.pwm3_khz  = f3;
    portEXIT_CRITICAL(&s_lock);
    mcpwm_drive_set_freq(f12, f3);
}

void pid_set_active_dof(uint8_t dof) {
    if (dof < 2 || dof > 3) return;
    portENTER_CRITICAL(&s_lock);
    s_cfg.active_dof = dof;
    if (dof == 2) {
        disable_motor_locked(2);
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_enable_motor(int motor_idx) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    float pos = s_rt[motor_idx].feedback_raw_12;
    const motor_cfg_t *mc = &s_cfg.motors[motor_idx];
    if (pos > (float)mc->cutoff_min_12 && pos < (float)mc->cutoff_max_12) {
        if (mcpwm_drive_enable(motor_idx)) {
            s_rt[motor_idx].disabled = false;
        }
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_enable_mask(uint8_t mask) {
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (mask & (1U << i)) {
            pid_enable_motor(i);
        }
    }
}

void pid_disable_all(void) {
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        disable_motor_locked(i);
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_hold_mask(uint8_t mask) {
    if (mask == 0 || (mask & ~0x07)) return;

    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (s_rt[i].jog.running) {
            portEXIT_CRITICAL(&s_lock);
            return;
        }
    }

    // Validate all requested motors sit safely inside clip & cutoff bounds
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (mask & (1U << i)) {
            float pos = s_rt[i].feedback_raw_12;
            const motor_cfg_t *mc = &s_cfg.motors[i];
            if (pos < (float)mc->clip_min_12 + 16.0f ||
                pos > (float)mc->clip_max_12 - 16.0f ||
                pos <= (float)mc->cutoff_min_12 ||
                pos >= (float)mc->cutoff_max_12) {
                for (int j = 0; j < SIMSYNC_NUM_MOTORS; ++j) {
                    disable_motor_locked(j);
                }
                portEXIT_CRITICAL(&s_lock);
                return;
            }
        }
    }

    // Atomically lock target == current feedback, reset PID history, and enable
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (mask & (1U << i)) {
            disable_motor_locked(i);
            float pos = s_rt[i].feedback_raw_12;
            s_rt[i].target_12        = pos;
            s_rt[i].last_target_12   = pos;
            s_rt[i].last_pos         = pos;
            s_rt[i].feedback_filt_12 = pos;
            s_rt[i].integrator       = 0.0f;
            s_rt[i].d_term           = 0.0f;
            s_rt[i].duty_out         = 0;
        }
    }
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (mask & (1U << i)) {
            if (mcpwm_drive_enable(i)) {
                s_rt[i].disabled = false;
            }
        }
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_jog_min_12(int motor_idx, uint16_t min_12) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    if (!s_rt[motor_idx].jog.running) {
        s_rt[motor_idx].jog.min_12 = clamp_u16(min_12, 0, 4095);
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_set_jog_max_12(int motor_idx, uint16_t max_12) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;
    portENTER_CRITICAL(&s_lock);
    if (!s_rt[motor_idx].jog.running) {
        s_rt[motor_idx].jog.max_12 = clamp_u16(max_12, 0, 4095);
    }
    portEXIT_CRITICAL(&s_lock);
}

void pid_start_jog(int motor_idx, int16_t duty_1000, uint16_t duration_ms) {
    if (motor_idx < 0 || motor_idx >= SIMSYNC_NUM_MOTORS) return;

    portENTER_CRITICAL(&s_lock);
    if (duty_1000 == 0) {
        if (s_rt[motor_idx].jog.running) {
            end_jog_locked(motor_idx);
        }
        portEXIT_CRITICAL(&s_lock);
        return;
    }

    // Only one open-loop jog may run, and all motors must currently be disabled
    for (int i = 0; i < SIMSYNC_NUM_MOTORS; ++i) {
        if (s_rt[i].jog.running || !s_rt[i].disabled) {
            portEXIT_CRITICAL(&s_lock);
            return;
        }
    }

    const motor_cfg_t *mc = &s_cfg.motors[motor_idx];
    motor_rt_t *rt        = &s_rt[motor_idx];
    int16_t abs_d = (duty_1000 < 0) ? -duty_1000 : duty_1000;
    float pos     = rt->feedback_raw_12;

    if (abs_d > 400 || abs_d > (int16_t)mc->pwm_max ||
        duration_ms < 50 || duration_ms > 500 ||
        rt->jog.min_12 >= rt->jog.max_12 ||
        rt->jog.min_12 <= mc->cutoff_min_12 ||
        rt->jog.max_12 >= mc->cutoff_max_12 ||
        pos <= (float)rt->jog.min_12 + 16.0f ||
        pos >= (float)rt->jog.max_12 - 16.0f) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }

    if (mcpwm_drive_enable(motor_idx)) {
        rt->disabled        = false;
        rt->jog.duty        = duty_1000;
        rt->jog.deadline_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000LL);
        rt->jog.running     = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

bool pid_clear_estop(void) {
    bool ok = mcpwm_drive_clear_estop();
    if (ok) {
        portENTER_CRITICAL(&s_lock);
        s_fault_flags &= ~FAULT_HW_ESTOP;
        portEXIT_CRITICAL(&s_lock);
    }
    return ok;
}

void pid_reset_timing_stats(void) {
    portENTER_CRITICAL(&s_lock);
    s_loop_max_us   = 0;
    s_jitter_max_us = 0;
    s_fault_flags  &= ~FAULT_ADC_OVERFLOW;
    portEXIT_CRITICAL(&s_lock);
}
