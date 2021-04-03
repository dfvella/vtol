#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#define MICROSEC_PER_SEC 1000000.0

#define PID_INITIAL_OUTPUT 1500
#define PID_MAX_OUTPUT 500

enum class Flight_Mode : uint8_t { TO_FORWARD, FORWARD, TO_SLOW, SLOW, TO_VERTICAL, VERTICAL };

class PIDcontroller
{
public:
    struct Gains
    {
        float p;
        float i;
        float d;
        float i_max;
    };

    PIDcontroller(const Gains& forward_in, const Gains& slow_in, 
                const Gains& vertical_in, const Flight_Mode& mode_in);
    float calculate(float error);

private:
    const Gains& select_gains();

    const Gains& forward_gains;
    const Gains& slow_gains;
    const Gains& vertical_gains;

    const Flight_Mode& flight_mode;

    float i_output, prev_output, prev_error;

    unsigned long timer;
};

#endif // PID_CONTROLLER_H
