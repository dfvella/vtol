#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "imu_driver.h"
#include "ppm_decoder.h"
#include "pwm_driver.h"
#include "control_map.h"
#include "pid_controller.h"

// Roll PID gains
#define ROLL_P 8 // was 10,12
#define ROLL_I 0
#define ROLL_D 0.5 // was 0
#define ROLL_I_MAX 1

// Pitch PID gains
#define PITCH_P 4 // was 6, 12
#define PITCH_I 8 // was 0
#define PITCH_D 0.5
#define PITCH_I_MAX 70 // was 1

// Yaw PID gains
#define YAW_P 1
#define YAW_I 0
#define YAW_D 0.1
#define YAW_I_MAX 1

// Three position switch thresholds
const uint16_t SWITCH_POS1 = 1300;
const uint16_t SWITCH_POS2 = 1700;

enum class Flight_Mode : uint8_t { FORWARD, SLOW, VERTICAL };
enum class Control_Mode : uint8_t { MANUAL, RATE, AUTOLEVEL };

class Flight_Controller {
public:
    Flight_Controller(Imu& imu, ppmDecoder& ppm);
    void ppmsync();
    void pidcalc();
    void servoset();
    void servowrite();

private:
    Imu& imu;
    ppmDecoder &ppm;

    PIDcontroller roll_pid{ROLL_P, ROLL_I, ROLL_D, ROLL_I_MAX};
    PIDcontroller pitch_pid{PITCH_P, PITCH_I, PITCH_D, PITCH_I_MAX};
    PIDcontroller yaw_pid{YAW_P, YAW_I, YAW_D, YAW_I_MAX};

    Flight_Mode flight_mode;
    Control_Mode control_mode;
};

#endif // FLIGHT_CONTROLLER_H
