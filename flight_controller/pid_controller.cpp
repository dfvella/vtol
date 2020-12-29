#include "pid_controller.h"

PIDcontroller::PIDcontroller(Gains gains_in, Flight_Mode& mode_in) 
    : gains{gains_in}, flight_mode{mode_in} { }

float PIDcontroller::calculate(float error)
{
    static bool start = true;

    float p, i, d, i_max;

    switch (flight_mode)
    {
    case Flight_Mode::VERTICAL:
        p = gains.p_vertical;
        i = gains.i_vertical;
        d = gains.d_vertical;
        i_max = gains.i_max_vertical;
        break;
    
    case Flight_Mode::SLOW:
        p = gains.p_slow;
        i = gains.i_slow;
        d = gains.d_slow;
        i_max = gains.i_max_slow;
        break;

    case Flight_Mode::FORWARD:
        p = gains.p_forward;
        i = gains.i_forward;
        d = gains.d_forward;
        i_max = gains.i_max_forward;
        break;
    }

    if (start)
    {
        timer = micros();
        start = false;

        return PID_INITIAL_OUTPUT;
    }
    else 
    {
        uint16_t t_delta = micros() - timer;
        timer = micros();

        float output = p * error;

        i_output += (t_delta / MICROSEC_PER_SEC) * error;
        i_output = constrain(i_output, (-1 * i_max) / i, i_max / i);

        output += constrain(i * i_output, -1 * i_max, i_max);
        output += d * ((error - prev_error) / t_delta) * MICROSEC_PER_SEC;
        prev_error = error;

        return constrain(output, -1 * PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    }
}
