#ifndef SIMSYNC_JOG_PULSE_H
#define SIMSYNC_JOG_PULSE_H
#include <stdint.h>

// Independent of Arduino so duration, bounds and rollover can be tested on host.
struct JogPulse {
    int16_t pwm = 0;
    uint16_t minimum = 512, maximum = 512;
    uint32_t deadline = 0;
    bool running = false;

    bool start(int16_t drive, uint16_t duration, int feedback, int ceiling, uint32_t now) {
        if (running || drive == 0 || drive < -100 || drive > 100 ||
            drive > ceiling || -drive > ceiling || duration < 50 || duration > 500 ||
            minimum >= maximum || maximum > 1023 ||
            feedback <= minimum + 4 || feedback >= maximum - 4) return false;
        pwm = drive;
        deadline = now + duration;
        running = true;
        return true;
    }

    bool update(int feedback, bool enabled, uint32_t now) {
        if (running && (!enabled || feedback <= minimum || feedback >= maximum ||
                        int32_t(now - deadline) >= 0)) running = false;
        return running;
    }
};
#endif
