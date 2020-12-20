#include "flight_controller.h"

Flight_Controller::Flight_Controller() { }

void Flight_Controller::begin()
{
    imu.calibrate();
}

void Flight_Controller::run()
{
    imu.run();
}

void Flight_Controller::calculate_outputs(Input& input, Output& output)
{
    determine_mode(input);

    Input pid_output;
    calculate_pids(input, pid_output);

    // Serial.print(pid_output.throttle);
    // Serial.print(' ');
    // Serial.print(pid_output.roll);
    // Serial.print(' ');
    // Serial.print(pid_output.pitch);
    // Serial.print(' ');
    // Serial.println(pid_output.yaw);

    map_outputs(pid_output, output);
}

void Flight_Controller::determine_mode(Input& input)
{
    flight_mode = Flight_Mode::VERTICAL;

    if (input.gear < SWITCH_POS2)
        flight_mode = Flight_Mode::SLOW;
    
    if (input.gear < SWITCH_POS1)
        flight_mode = Flight_Mode::FORWARD;

    control_mode = Control_Mode::AUTOLEVEL;

    if (input.aux < SWITCH_POS2)
        control_mode = Control_Mode::RATE;
    
    if (input.aux < SWITCH_POS1)
        control_mode = Control_Mode::MANUAL;
}

void Flight_Controller::calculate_pids(Input& input, Input& output)
{
    calculate_targets(input);

    switch (control_mode)
    {
    case Control_Mode::MANUAL:
        output = input;
        break;
    default:
        // add seperate pid controllers for different flight modes
        output = {
            input.throttle,
            (uint16_t)roll_pid.calculate(imu.roll() - target_roll + ROLL_PID_TRIM) + NUETRAL_STICK,
            (uint16_t)pitch_pid.calculate(imu.pitch() - target_pitch + PITCH_PID_TRIM) + NUETRAL_STICK,
            (uint16_t)yaw_pid.calculate(imu.yaw() - target_yaw + YAW_PID_TRIM) + NUETRAL_STICK,
            input.gear,
            input.aux
        };
        break;
    }
}

void Flight_Controller::calculate_targets(Input& input)
{
    if (abs(NUETRAL_STICK - (int16_t)input.roll) < DEAD_STICK)
        input.roll = NUETRAL_STICK;
    if (abs(NUETRAL_STICK - (int16_t)input.pitch) < DEAD_STICK)
        input.pitch = NUETRAL_STICK;
    if (abs(NUETRAL_STICK - (int16_t)input.yaw) < DEAD_STICK)
        input.yaw = NUETRAL_STICK;

    switch (control_mode)
    {
    case Control_Mode::AUTOLEVEL:
        target_roll = interpolate(input.roll, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * AUTO_MAX_ROLL_ANGLE, AUTO_MAX_ROLL_ANGLE);
        target_pitch = interpolate(input.pitch, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * AUTO_MAX_PITCH_ANGLE, AUTO_MAX_PITCH_ANGLE);
        target_yaw += interpolate(input.yaw, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_YAW_RATE, RATE_MAX_YAW_RATE);
        break;

    case Control_Mode::RATE:
        target_roll += interpolate(input.roll, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_ROLL_RATE, RATE_MAX_ROLL_RATE);
        target_pitch += interpolate(input.pitch, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_PITCH_RATE, RATE_MAX_PITCH_RATE);
        target_yaw += interpolate(input.yaw, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_YAW_RATE, RATE_MAX_YAW_RATE);
        break;

    case Control_Mode::MANUAL:
        target_roll = 0;
        target_pitch = 0;
        target_yaw = 0;
        break;
    }

    // currently inverted flight not possible in rate mode
    if (target_pitch > 60)
        target_pitch = 60;
    else if (target_pitch < -60)
        target_pitch = -60;

    if (target_roll > 180)
        target_roll -= 360;
    else if (target_roll < -180)
        target_roll += 360;

    if (target_yaw > 180)
        target_yaw -= 360;
    else if (target_yaw < -180)
        target_yaw += 360;
}

void Flight_Controller::map_outputs(Input& input, Output& output)
{
    switch (flight_mode)
    {
    case Flight_Mode::FORWARD:
        output.right_motor = input.throttle + (FORWARD_YAW_DIFFERENTIAL * (NUETRAL_STICK - (int16_t)input.yaw));
        output.left_motor = input.throttle - (FORWARD_YAW_DIFFERENTIAL * (NUETRAL_STICK - (int16_t)input.yaw));

        output.right_alr = input.roll;
        output.left_alr = input.roll;

        output.right_tilt = FORWARD_RIGHT_TILT;
        output.left_tilt = FORWARD_LEFT_TILT;

        output.elevator = input.pitch;
        break;
    
    case Flight_Mode::SLOW:
        // add aileron into mix thrust differential
        output.right_motor = input.throttle + (SLOW_YAW_DIFFERENTIAL * (NUETRAL_STICK - (int16_t)input.yaw));
        output.left_motor = input.throttle - (SLOW_YAW_DIFFERENTIAL * (NUETRAL_STICK - (int16_t)input.yaw));

        output.right_alr = input.roll + SLOW_FLAPS_TRIM;
        output.left_alr = input.roll - SLOW_FLAPS_TRIM;

        // add elevator into tilt
        output.right_tilt = SLOW_RIGHT_TILT;
        output.left_tilt = SLOW_LEFT_TILT;

        output.elevator = input.pitch;
        break;

    case Flight_Mode::VERTICAL:
        output.right_motor = input.throttle + (VERTICAL_ROLL_DIFFERENTIAL * (NUETRAL_STICK - (int16_t)input.roll));
        output.left_motor = input.throttle - (VERTICAL_ROLL_DIFFERENTIAL * (NUETRAL_STICK - (int16_t)input.roll));

        output.right_alr = VERTICAL_FLAPS_TRIM;
        output.left_alr = VERTICAL_FLAPS_TRIM;

        output.right_tilt = VERTICAL_RIGHT_TILT + (VERTICAL_YAW_MOTOR_TILT * (NUETRAL_STICK - (int16_t)input.yaw))
                                                + (VERTICAL_PITCH_MOTOR_TILT * (NUETRAL_STICK - (int16_t)input.pitch));
        output.left_tilt = VERTICAL_LEFT_TILT - (VERTICAL_YAW_MOTOR_TILT * (NUETRAL_STICK - (int16_t)input.yaw))
                                              - (VERTICAL_PITCH_MOTOR_TILT * (NUETRAL_STICK - (int16_t)input.yaw));

        output.elevator = input.pitch;
        break;
    }
}

float Flight_Controller::get_target_roll()
{
    return target_roll;
}

float Flight_Controller::get_target_pitch()
{
    return target_pitch;
}

float Flight_Controller::get_target_yaw()
{
    return target_yaw;
}

float Flight_Controller::get_roll_angle()
{
    return imu.roll();
}

float Flight_Controller::get_pitch_angle()
{
    return imu.pitch();
}

float Flight_Controller::get_yaw_angle()
{
    return imu.yaw();
}

Control_Mode Flight_Controller::get_control_mode()
{
    return control_mode;
}

Flight_Mode Flight_Controller::get_flight_mode()
{
    return flight_mode;
}

float Flight_Controller::interpolate(float val, float min_from, float max_from, float min_to, float max_to)
{    
    return (((val - min_from) / (max_from - min_from)) * (max_to - min_to)) + min_to;
}
