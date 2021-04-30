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
    // get the requested flight mode
    Flight_Mode target_flight_mode = Flight_Mode::VERTICAL;

    if (input.gear < SWITCH_POS2)
        target_flight_mode = Flight_Mode::SLOW;
    
    if (input.gear < SWITCH_POS1)
        target_flight_mode = Flight_Mode::FORWARD;

    // get the necessary flight mode
    switch (flight_mode)
    {
    case Flight_Mode::TO_FORWARD:
        if (target_flight_mode != Flight_Mode::FORWARD)
            flight_mode = Flight_Mode::TO_SLOW;
        else if (transition_state == -TRANSITION_TIME)
            flight_mode = Flight_Mode::FORWARD;
        break;

    case Flight_Mode::FORWARD:
        if (target_flight_mode != Flight_Mode::FORWARD)
            flight_mode = Flight_Mode::TO_SLOW;
        break;

    case Flight_Mode::TO_SLOW:
        if (target_flight_mode == Flight_Mode::FORWARD && transition_state < 0)
                flight_mode = Flight_Mode::TO_FORWARD;
        else if (target_flight_mode == Flight_Mode::VERTICAL && transition_state > 0)
            flight_mode = Flight_Mode::TO_VERTICAL;
        else if (transition_state == 0)
            flight_mode = Flight_Mode::SLOW;
        break;

    case Flight_Mode::SLOW:
        if (target_flight_mode == Flight_Mode::FORWARD)
            flight_mode = Flight_Mode::TO_FORWARD;

        if (target_flight_mode == Flight_Mode::VERTICAL)
            flight_mode = Flight_Mode::TO_VERTICAL;
        break;

    case Flight_Mode::TO_VERTICAL:
        if (target_flight_mode != Flight_Mode::VERTICAL)
            flight_mode = Flight_Mode::TO_SLOW;
        else if (transition_state == TRANSITION_TIME)
            flight_mode = Flight_Mode::VERTICAL;
        break;

    case Flight_Mode::VERTICAL:
        if (target_flight_mode != Flight_Mode::VERTICAL)
            flight_mode = Flight_Mode::TO_SLOW;
        break;
    }

    // update transition state
    switch (flight_mode)
    {
    case Flight_Mode::TO_FORWARD:
        --transition_state;
        break;

    case Flight_Mode::FORWARD:
        transition_state = -TRANSITION_TIME;
        break;

    case Flight_Mode::TO_SLOW:
        if (transition_state > 0)
            --transition_state;
        else if (transition_state < 0)
            ++transition_state;
        break;

    case Flight_Mode::SLOW:
        transition_state = 0;
        break;

    case Flight_Mode::TO_VERTICAL:
        ++transition_state;
        break;

    case Flight_Mode::VERTICAL:
        transition_state = TRANSITION_TIME;
        break;
    }



    control_mode = Control_Mode::AUTOLEVEL;

    if (input.aux < SWITCH_POS2)
        control_mode = Control_Mode::RATE;
    
    if (input.aux < SWITCH_POS1)
        control_mode = Control_Mode::MANUAL;
}

void Flight_Controller::calculate_pids(Input& input, Input& output)
{
    float roll_error = imu.roll() - target_roll;
    float pitch_error = imu.pitch() - target_pitch;
    float yaw_error = imu.yaw() - target_yaw;

    if (roll_error > 180)
        roll_error -= 360;
    else if (roll_error < -180)
        roll_error += 360;

    if (pitch_error > 180)
        pitch_error -= 360;
    else if (pitch_error < -180)
        pitch_error += 360;

    if (yaw_error > 180)
        yaw_error -= 360;
    else if (yaw_error < -180)
        yaw_error += 360;

    calculate_targets(input);

    output.throttle = input.throttle;

    switch (control_mode)
    {
    case Control_Mode::MANUAL:
        output.roll = input.roll;
        output.pitch = input.pitch;
        output.yaw = input.yaw;
        break;

    default:
        output.roll = (uint16_t)roll_pid.calculate(roll_error + ROLL_PID_TRIM) + NEUTRAL_STICK;
        output.pitch = (uint16_t)pitch_pid.calculate(pitch_error + PITCH_PID_TRIM) + NEUTRAL_STICK;
        output.yaw = (uint16_t)yaw_pid.calculate(yaw_error + YAW_PID_TRIM) + NEUTRAL_STICK;
        break;
    }
}

void Flight_Controller::calculate_targets(Input& input)
{
    if (abs(NEUTRAL_STICK - (int16_t)input.roll) < DEAD_STICK)
        input.roll = NEUTRAL_STICK;
    if (abs(NEUTRAL_STICK - (int16_t)input.pitch) < DEAD_STICK)
        input.pitch = NEUTRAL_STICK;
    if (abs(NEUTRAL_STICK - (int16_t)input.yaw) < DEAD_STICK)
        input.yaw = NEUTRAL_STICK;

    switch (control_mode)
    {
    case Control_Mode::AUTOLEVEL:
        target_roll = -1 * interpolate(input.roll, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * AUTO_MAX_ROLL_ANGLE, AUTO_MAX_ROLL_ANGLE);
        target_pitch = -1 * interpolate(input.pitch, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * AUTO_MAX_PITCH_ANGLE, AUTO_MAX_PITCH_ANGLE);
        target_yaw += -1 * interpolate(input.yaw, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_YAW_RATE, RATE_MAX_YAW_RATE);
        break;

    case Control_Mode::RATE:
        target_roll += -1 * interpolate(input.roll, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_ROLL_RATE, RATE_MAX_ROLL_RATE);
        target_pitch += -1 * interpolate(input.pitch, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_PITCH_RATE, RATE_MAX_PITCH_RATE);
        target_yaw += -1 * interpolate(input.yaw, MIN_PWM_PULSEWIDTH, MAX_PWM_PULSEWIDTH, -1 * RATE_MAX_YAW_RATE, RATE_MAX_YAW_RATE);
        break;

    case Control_Mode::MANUAL:
        target_roll = imu.roll();
        target_pitch = imu.pitch();
        target_yaw = imu.yaw();
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
    // set the output motor tilt based on flight mode
    if (flight_mode == Flight_Mode::TO_FORWARD || flight_mode == Flight_Mode::FORWARD)
    {
        output.right_tilt = FORWARD_RIGHT_TILT;
        output.left_tilt = FORWARD_LEFT_TILT;
    }
    else if (flight_mode == Flight_Mode::TO_SLOW || flight_mode == Flight_Mode::SLOW)
    {
        output.right_tilt = SLOW_RIGHT_TILT;
        output.left_tilt = SLOW_LEFT_TILT;
    }
    else
    {
        output.right_tilt = VERTICAL_RIGHT_TILT;
        output.left_tilt = VERTICAL_LEFT_TILT;
    }

    // set other outputs based on current motor tilt
    if (transition_state < 0)
    // currently in, or transitioning to or from forward flight
    {
        output.right_motor = input.throttle - (FORWARD_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw));
        output.left_motor = input.throttle + (FORWARD_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw));

        output.right_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll;
        output.left_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll;

        output.elevator = NEUTRAL_STICK + (NEUTRAL_STICK - (int16_t)input.pitch);
    }
    else if (0 <= transition_state && transition_state < TRANSITION_TIME)
    // currently in slow flight or transitioning to or from vertical flight
    {
        // add aileron into mix thrust differential
        output.right_motor = input.throttle - (SLOW_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw));
        output.left_motor = input.throttle + (SLOW_YAW_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.yaw));

        output.right_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll + SLOW_FLAPS_TRIM;
        output.left_alr = (2 * NEUTRAL_STICK) - (int16_t)input.roll - SLOW_FLAPS_TRIM;

        // add elevator into tilt?
        //output.right_tilt = SLOW_RIGHT_TILT;
        //output.left_tilt = SLOW_LEFT_TILT;

        output.elevator = NEUTRAL_STICK + (NEUTRAL_STICK - (int16_t)input.pitch);
    }
    else // currently in vertical flight mode
    {
        output.right_motor = input.throttle - (VERTICAL_ROLL_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.roll));
        output.left_motor = input.throttle + (VERTICAL_ROLL_DIFFERENTIAL * (NEUTRAL_STICK - (int16_t)input.roll));

        output.right_alr = NEUTRAL_STICK + VERTICAL_FLAPS_TRIM;
        output.left_alr = NEUTRAL_STICK - VERTICAL_FLAPS_TRIM;

        output.right_tilt += (VERTICAL_YAW_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.yaw));
        output.right_tilt += (VERTICAL_PITCH_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.pitch));
        output.right_tilt -= VERTICAL_YAW_TRIM;

        output.left_tilt += (VERTICAL_YAW_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.yaw));
        output.left_tilt -= (VERTICAL_PITCH_MOTOR_TILT * (NEUTRAL_STICK - (int16_t)input.pitch));
        output.left_tilt -= VERTICAL_YAW_TRIM;

        output.elevator = NEUTRAL_STICK + (NEUTRAL_STICK - (int16_t)input.pitch);
    }

    // for safety: make sure only to spin up motors when throttle is applied.
    // without this flight controller could command motors for control about another axis
    // even if there is no throttle input.
    if (abs(input.throttle - NEUTRAL_THROTTLE) < DEAD_STICK)
    {
        output.right_motor = RIGHT_MOTOR_MIN_PULSE;
        output.left_motor = LEFT_MOTOR_MIN_PULSE;
    }

    // if (output.elevator < CENTER_ELEVATOR)
    //     output.elevator = interpolate(output.elevator, MIN_PWM_PULSEWIDTH, DEAD_STICK, MIN_ELEVATOR_THROW, CENTER_ELEVATOR);
    // else
    //     output.elevator = interpolate(output.elevator, DEAD_STICK, MAX_PWM_PULSEWIDTH, CENTER_ELEVATOR, MAX_ELEVATOR_THROW);

    // if (output.right_alr < CENTER_RIGHT_ALR)
    //     output.right_alr = interpolate(output.right_alr, MIN_PWM_PULSEWIDTH, DEAD_STICK, MIN_RIGHT_ALR_THROW, CENTER_RIGHT_ALR);
    // else
    //     output.right_alr = interpolate(output.right_alr, DEAD_STICK, MAX_PWM_PULSEWIDTH, CENTER_RIGHT_ALR, MAX_RIGHT_ALR_THROW);

    // if (output.left_alr < CENTER_LEFT_ALR)
    //     output.left_alr = interpolate(output.left_alr, MIN_PWM_PULSEWIDTH, DEAD_STICK, MIN_LEFT_ALR_THROW, CENTER_LEFT_ALR);
    // else
    //     output.left_alr = interpolate(output.left_alr, DEAD_STICK, MAX_PWM_PULSEWIDTH, CENTER_LEFT_ALR, MAX_LEFT_ALR_THROW);


    // dampen the tilt servos when transitioning between flight modes
    if (flight_mode == Flight_Mode::TO_FORWARD ||
        flight_mode == Flight_Mode::TO_SLOW ||
        flight_mode == Flight_Mode::TO_VERTICAL)
    {
        uint16_t right_tilt_damper;
        uint16_t left_tilt_damper;

        if (transition_state < 0) // tile position currently between slow and forward
        {
            right_tilt_damper = (SLOW_RIGHT_TILT - FORWARD_RIGHT_TILT) / TRANSITION_TIME;
            left_tilt_damper = (FORWARD_LEFT_TILT - SLOW_LEFT_TILT) / TRANSITION_TIME;
        }
        else
        {
            right_tilt_damper = (VERTICAL_RIGHT_TILT - SLOW_RIGHT_TILT) / TRANSITION_TIME;
            left_tilt_damper = (SLOW_LEFT_TILT - VERTICAL_LEFT_TILT) / TRANSITION_TIME;
        }

        output.right_tilt = constrain(output.right_tilt, right_tilt_last - right_tilt_damper, right_tilt_last + right_tilt_damper);
        output.left_tilt = constrain(output.left_tilt, left_tilt_last - left_tilt_damper, left_tilt_last + left_tilt_damper);
    }
    right_tilt_last = output.right_tilt;
    left_tilt_last = output.left_tilt;
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

int16_t Flight_Controller::get_transition_state()
{
    return transition_state;
}

float Flight_Controller::interpolate(float val, float min_from, float max_from, float min_to, float max_to)
{    
    return (((val - min_from) / (max_from - min_from)) * (max_to - min_to)) + min_to;
}
