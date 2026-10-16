#ifndef SIMSYNC_PID_FILTERS_H
#define SIMSYNC_PID_FILTERS_H

// Fast controller filters. HPF is ONLY in the derivative branch, never DC position.
struct PIDFilters {
    unsigned int lpfTenths = 0, hpfTenths = 0;
    float position = 0, previousDelta = 0, highPassedDelta = 0;
    float lpAlpha = 1, hpAlpha = 1, lpDt = -1, hpDt = -1;
    bool initialized = false;
    void setLPF(unsigned int value) { lpfTenths=value; lpDt=-1; initialized=false; }
    void setHPF(unsigned int value) { hpfTenths=value; hpDt=-1; previousDelta=0; highPassedDelta=0; }
    float feedback(float raw, float dt, bool disabled) {
        if (!initialized || disabled) {
            position=raw; initialized=true; previousDelta=0; highPassedDelta=0;
            return raw;
        }
        if (!lpfTenths) { position=raw; return raw; }
        if (lpDt!=dt) {
            float wdt=0.62831853f*lpfTenths*dt;
            lpAlpha=wdt/(1+wdt); lpDt=dt;
        }
        position += lpAlpha*(raw-position);
        return position;
    }
    float derivative(float delta, float dt) {
        if (!hpfTenths) { previousDelta=delta; return delta; }
        if (hpDt!=dt) { hpAlpha=1/(1+0.62831853f*hpfTenths*dt); hpDt=dt; }
        highPassedDelta=hpAlpha*(highPassedDelta+delta-previousDelta);
        previousDelta=delta;
        return highPassedDelta;
    }
};
#endif
