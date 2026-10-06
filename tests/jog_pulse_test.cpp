#include "../SMC3/JogPulse.h"
#include <assert.h>

int main() {
    JogPulse j;
    j.minimum=480; j.maximum=544;
    assert(!j.start(40,150,512,30,1000));
    assert(!j.start(40,501,512,40,1000));
    assert(!j.start(40,150,484,40,1000));
    assert(j.start(-40,150,512,40,1000));
    assert(j.pwm==-40);
    assert(!j.start(40,500,512,40,1010)); // No pulse extension.
    assert(j.update(512,true,1149));
    assert(!j.update(512,true,1150));
    assert(j.start(40,150,512,40,2000));
    assert(!j.update(544,true,2010));
    assert(j.start(40,150,512,40,3000));
    assert(!j.update(480,true,3010));
    assert(j.start(40,150,512,40,4000));
    assert(!j.update(512,false,4010));
    assert(j.start(40,150,512,40,UINT32_MAX-99));
    assert(j.update(512,true,49));
    assert(!j.update(512,true,50));
    return 0;
}
