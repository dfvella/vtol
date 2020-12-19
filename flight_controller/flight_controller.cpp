#include "flight_controller.h"

Flight_Controller::Flight_Controller(Imu& imu_in, ppmDecoder& ppm_in)
    : imu{imu_in}, ppm{ppm_in} { }

void Flight_Controller::ppmsync()
{
    flight_mode = Flight_Mode::VERTICAL;

    if (ppm.get(ppmDecoder::GER) < SWITCH_POS2)
        flight_mode = Flight_Mode::SLOW;
    
    if (ppm.get(ppmDecoder::GER) < SWITCH_POS1)
        flight_mode = Flight_Mode::FORWARD;

    control_mode = Control_Mode::AUTOLEVEL;

    if (ppm.get(ppmDecoder::AUX) < SWITCH_POS2)
        control_mode = Control_Mode::RATE;
    
    if (ppm.get(ppmDecoder::AUX) < SWITCH_POS1)
        control_mode = Control_Mode::MANUAL;

    ppm.sync();
}

void Flight_Controller::pidcalc()
{

}

void Flight_Controller::servoset()
{

}

void Flight_Controller::servowrite()
{
    
}
