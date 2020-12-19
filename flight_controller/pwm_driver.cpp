#include "pwm_driver.h"

pwmDevice::pwmDevice(uint8_t pin_in)
    : pin{pin_in}, min_signal{1100}, max_signal{1900} { }

pwmDevice::pwmDevice(uint8_t pin_in, uint16_t min_in, uint16_t max_in)
    : pin{pin_in}, min_signal{min_in}, max_signal{max_in} { }

void pwmDevice::set_signal(uint16_t signal_in)
{
    if (abs(signal_in - signal) > NOISE_FILTER_THRESHOLD)
        signal = signal_in;

    signal = constrain(signal, min_signal, max_signal);
}

uint16_t pwmDevice::get_signal()
{
    return signal;
}

void pwmDevice::begin()
{
    pinMode(pin, OUTPUT);
}

void pwmDevice::write(uint16_t signal_in)
{
    high();
    low();
}

void pwmDevice::high()
{
    digitalWrite(pin, HIGH);
    timer = micros();
}

void pwmDevice::low()
{
    while (micros() - timer < signal);
    digitalWrite(pin, LOW);
}

pwmScheduler::pwmScheduler() { }

void pwmScheduler::add_device(uint8_t ind, pwmDevice* device)
{
    sorted_devices[ind] = device;
    devices[ind] = device;
    devices[ind]->begin();
}

void pwmScheduler::set_signal(uint8_t ind, uint16_t signal)
{
    devices[ind]->set_signal(signal);
}

void pwmScheduler::write_all()
{
    uint8_t correct = 0;

    // Bubble sort the array according to signal pulsewidth
    while (correct != NUM_DEVICES - 1)
    {
        correct = 0;
        for (uint8_t i = 0; i < NUM_DEVICES - 1; ++i)
        {
            if (sorted_devices[i]->get_signal() <= sorted_devices[i + 1]->get_signal())
                ++correct;
            else 
            {
                pwmDevice* temp = sorted_devices[i];
                sorted_devices[i] = sorted_devices[i + 1];
                sorted_devices[i + 1] = temp;
            }
        }
    }

    for (uint8_t i = 0; i < NUM_DEVICES; ++i)
    {
        sorted_devices[i]->high();
        unsigned long temp = micros();
        while (micros() - temp < PULSE_PADDING);
    }

    // Write all the servo output pins low
    for (uint8_t i = 0; i < NUM_DEVICES; ++i) 
        sorted_devices[i]->low();
}



// void Servo::write_all(Servo* servo[], const uint8_t num)
// {
//     // Make a copt of the servo array
//     Servo* sorted_servo[num];

//     for (uint8_t i = 0; i < num; ++i)
//         sorted_servo[i] = servo[i];
    
//     uint8_t correct = 0;

//     // Bubble sort the array according to signal pulsewidth
//     while (correct != num - 1)
//     {
//         correct = 0;
//         for (uint8_t i = 0; i + 1 != num; ++i)
//         {
//             if (sorted_servo[i]->signal <= sorted_servo[i + 1]->signal)
//                 ++correct;
//             else 
//             {
//                 Servo* servo_temp = sorted_servo[i];
//                 sorted_servo[i] = sorted_servo[i + 1];
//                 sorted_servo[i + 1] = servo_temp;
//             }
//         }
//     }
    
//     // Write all the pins high with padding according to PULSE_PADDING
//     // between rising edges 
//     for (uint8_t i = 0; i < num; ++i)
//     {
//         sorted_servo[i]->high();
//         unsigned long temp = micros();
//         while (micros() - temp < PULSE_PADDING);
//     }

//     // Write all the servo output pins low
//     for (uint8_t i = 0; i < num; ++i) 
//         sorted_servo[i]->low();
// }
