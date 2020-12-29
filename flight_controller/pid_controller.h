#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#define MICROSEC_PER_SEC 1000000.0

#define PID_INITIAL_OUTPUT 1500
#define PID_MAX_OUTPUT 500

enum class Flight_Mode : uint8_t { FORWARD, SLOW, VERTICAL };

class PIDcontroller
{
public:
    struct Gains
    {
        float p_forward, i_forward, d_forward, i_max_forward;
        float p_slow, i_slow, d_slow, i_max_slow;
        float p_vertical, i_vertical, d_vertical, i_max_vertical;
    };

    PIDcontroller(Gains gains_in, Flight_Mode& mode_in);
    float calculate(float error);

private:
    Gains gains;

    Flight_Mode& flight_mode;

    float i_output, prev_output, prev_error;

    unsigned long timer;
};

#endif // PID_CONTROLLER_H
