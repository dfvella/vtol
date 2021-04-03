#include "pid_controller.h"

PIDcontroller::PIDcontroller(const Gains& forward_in, const Gains& slow_in,
                            const Gains& vertical_in, const Flight_Mode& mode_in) 
    : forward_gains{forward_in},
        slow_gains{slow_in},
        vertical_gains{vertical_in}, 
        flight_mode{mode_in} { }

float PIDcontroller::calculate(float error)
{
    static bool start = true;

    const Gains& gains = select_gains();

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

        float output = gains.p * error;

        i_output += (t_delta / MICROSEC_PER_SEC) * error;
        i_output = constrain(i_output, (-1 * gains.i_max) / gains.i, gains.i_max / gains.i);

        output += constrain(gains.i * i_output, -1 * gains.i_max, gains.i_max);
        output += gains.d * ((error - prev_error) / t_delta) * MICROSEC_PER_SEC;
        prev_error = error;

        return constrain(output, -1 * PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    }
}

const PIDcontroller::Gains& PIDcontroller::select_gains()
{
    if (flight_mode == Flight_Mode::FORWARD ||
        flight_mode == Flight_Mode::TO_SLOW ||
        flight_mode == Flight_Mode::TO_FORWARD)
    {
        return forward_gains;
    }
    else if (flight_mode == Flight_Mode::SLOW ||
        flight_mode == Flight_Mode::TO_VERTICAL)
    {
        return slow_gains;
    }
    return vertical_gains;
}
