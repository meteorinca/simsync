#include "../PIDFilters.h"
#include <cassert>
#include <cmath>

int main() {
    PIDFilters first, second;
    for (int i=0; i<100; ++i) {
        assert(first.feedback(338+i,0.00025f,false)==338+i);
        assert(first.derivative(i,0.00025f)==i);
    }
    first.setLPF(800); // 80 Hz; reject high-frequency ADC chatter.
    first.feedback(338,0.00025f,true);
    float outputEnergy=0;
    for (int i=0; i<4000; ++i) {
        float raw=338+std::sin(2*3.14159265f*1000*i*0.00025f);
        float output=first.feedback(raw,0.00025f,false)-338;
        if(i>=2000) outputEnergy+=output*output;
    }
    assert(std::sqrt(outputEnergy/2000)<0.1f);
    assert(second.feedback(400,0.00025f,false)==400); // Channel isolation.
    // LPF preserves DC and resets to exact feedback on disabled/hold.
    for (int i=0; i<4000; ++i) first.feedback(500,0.00025f,false);
    assert(std::fabs(first.position-500)<0.001f);
    assert(first.feedback(338,0.00025f,true)==338);
    first.setHPF(20); // 2 Hz, only on the D branch.
    float last=0;
    for (int i=0; i<4000; ++i) last=first.derivative(1,0.001f);
    assert(std::fabs(last)<0.0001f); // Constant derivative component washes out.
    assert(std::fabs(first.feedback(338,0.00025f,false)-338)<0.001f);
    first.setHPF(0);
    assert(first.derivative(3,0.001f)==3);
    // Same cutoff has the same time response at different sampling intervals.
    PIDFilters fast, slow;
    fast.setLPF(100); slow.setLPF(100);
    fast.feedback(0,0.00025f,true); slow.feedback(0,0.0025f,true);
    for(int i=0;i<400;++i) fast.feedback(1,0.00025f,false);
    for(int i=0;i<40;++i) slow.feedback(1,0.0025f,false);
    assert(std::fabs(fast.position-slow.position)<0.003f);
}
