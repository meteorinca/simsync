#ifndef SIMSYNC_MOTOR_DRIVE_H
#define SIMSYNC_MOTOR_DRIVE_H
// MODE2 uses a direction level on RPWM and complementary PWM on LPWM.
// Invert the physical drive, never the feedback coordinate or target.
struct MotorDrive {
    bool high;
    int duty;
};
inline MotorDrive mode2Drive(int drive, bool reversed) {
    if (reversed) drive = -drive;
    return {drive > 0, drive > 0 ? 255-drive : -drive};
}
#endif
