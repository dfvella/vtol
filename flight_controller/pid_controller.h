#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#define MICROSEC_PER_SEC 1000000.0
#define PID_INITIAL_OUTPUT 1500

class PIDcontroller
{
    public:
        PIDcontroller(float p_in, float i_in, float d_in, float i_max_in);
        float calculate(float error);

    private:
        float p, i, d, i_max;
        float i_output, prev_output, prev_error;

        unsigned long timer;
};

#endif // PID_CONTROLLER_H
