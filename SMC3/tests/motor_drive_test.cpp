#include "../MotorDrive.h"
#include <cassert>
int main() {
    for (int drive=-255;drive<=255;++drive) {
        MotorDrive normal=mode2Drive(drive,false);
        MotorDrive reversed=mode2Drive(drive,true);
        // Signed differential bridge duty, RPWM minus LPWM.
        assert((normal.high?255:0)-normal.duty==drive);
        assert((reversed.high?255:0)-reversed.duty==-drive);
        assert(normal.duty>=0 && normal.duty<=255);
        assert(reversed.duty>=0 && reversed.duty<=255);
    }
}
