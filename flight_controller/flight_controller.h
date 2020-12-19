#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>

#include "imu_driver.h"
#include "pid_controller.h"

// forward control map gains
#define FORWARD_YAW_DIFFERENTIAL 0.1
#define FORWARD_RIGHT_TILT 1000
#define FORWARD_LEFT_TILT 1000

// slow control map gains
#define SLOW_YAW_DIFFERENTIAL 0.05
#define SLOW_ROLL_DIFFERENTIAL 0.05
#define SLOW_FLAPS_TRIM 100
#define SLOW_RIGHT_TILT 1500
#define SLOW_LEFT_TILT 1500

// verital control map gains
#define VERTICAL_ROLL_DIFFERENTIAL 0.01
#define VERTICAL_YAW_MOTOR_TILT 0.1
#define VERTICAL_FLAPS_TRIM 100
#define VERTICAL_RIGHT_TILT 2000
#define VERTICAL_LEFT_TILT 2000


#define AUTO_MAX_ROLL_ANGLE 40
#define AUTO_MAX_PITCH_ANGLE 40

#define RATE_MAX_ROLL_RATE 3.6 // units: deg per 20 ms
#define RATE_MAX_PITCH_RATE 3.6
#define RATE_MAX_YAW_RATE 3.6

// Roll PID gains
#define ROLL_P 8 // was 10, 12
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

// trim
#define ROLL_PID_TRIM 0
#define PITCH_PID_TRIM 0
#define YAW_PID_TRIM 0

// Three position switch thresholds
#define SWITCH_POS1 1300
#define SWITCH_POS2 1700

#define MIN_PWM_PULSEWIDTH 1000
#define MAX_PWM_PULSEWIDTH 2000

enum class Flight_Mode : uint8_t { FORWARD, SLOW, VERTICAL };
enum class Control_Mode : uint8_t { MANUAL, RATE, AUTOLEVEL };

class Flight_Controller {
public:
    Flight_Controller();
    void begin();
    void run();

    struct Input
    {
        uint16_t throttle, roll, pitch, yaw, gear, aux;
    };

    struct Output
    {
        uint16_t right_motor, right_tilt, right_alr;
        uint16_t left_motor, left_tilt, left_alr;
        uint16_t elevator;
    };

    void calculate_outputs(Input& input, Output& output);

    float get_target_roll();
    float get_target_pitch();
    float get_target_yaw();

private:
    void determine_mode(Input& input);
    void calculate_pids(Input& input, Input& output);
    void calculate_targets(Input& input);
    void map_outputs(Input& input, Output& output);

    float interpolate(float val, float min_from, float max_from, float min_to, float max_to);

    Imu imu{ LED_BUILTIN };

    // add seperate pid controllers for different flight modes
    PIDcontroller roll_pid{ ROLL_P, ROLL_I, ROLL_D, ROLL_I_MAX };
    PIDcontroller pitch_pid{ PITCH_P, PITCH_I, PITCH_D, PITCH_I_MAX };
    PIDcontroller yaw_pid{ YAW_P, YAW_I, YAW_D, YAW_I_MAX };

    float target_roll = 0;
    float target_pitch = 0;
    float target_yaw = 0;

    Flight_Mode flight_mode;
    Control_Mode control_mode;
};

#endif // FLIGHT_CONTROLLER_H
