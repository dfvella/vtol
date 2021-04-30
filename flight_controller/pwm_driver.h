#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <Arduino.h>

// pwm signal max and min pulsewidths
#define RIGHT_MOTOR_MIN_PULSE 1100
#define RIGHT_MOTOR_MAX_PULSE 1900

#define RIGHT_TILT_MIN_PULSE 910
#define RIGHT_TILT_MAX_PULSE 2050

#define RIGHT_ALR_MIN_PULSE 1210
#define RIGHT_ALR_MAX_PULSE 1850

#define LEFT_MOTOR_MIN_PULSE 1100
#define LEFT_MOTOR_MAX_PULSE 1900

#define LEFT_TILT_MIN_PULSE 1150
#define LEFT_TILT_MAX_PULSE 2150

#define LEFT_ALR_MIN_PULSE 1200
#define LEFT_ALR_MAX_PULSE 1710

#define ELE_MIN_PULSE 1200
#define ELE_MAX_PULSE 1800

class pwmDevice
{
public:
    pwmDevice(uint8_t pin_in);
    pwmDevice(uint8_t pin_in, uint16_t min_in, uint16_t max_in);

    void set_signal(uint16_t signal_in);
    uint16_t get_signal();

    void begin();
    void write(uint16_t signal_in);

    // Minimum difference required for the signal property to update
    // upon a call to set() method. For signal noise reduction.
    static constexpr uint8_t NOISE_FILTER_THRESHOLD = 10; // microseconds

private:
    friend class pwmScheduler;

    void high();
    void low();

    uint8_t pin;
    uint16_t signal;

    uint16_t min_signal;
    uint16_t max_signal;

    unsigned long timer;
};

class pwmScheduler
{
public:
    pwmScheduler();

    void add_device(uint8_t ind, pwmDevice* device);
    void set_signal(uint8_t ind, uint16_t signal);
    void write_all();

    static constexpr uint8_t RESC_IND = 0;
    static constexpr uint8_t RTILT_IND = 1;
    static constexpr uint8_t RALR_IND = 2;
    static constexpr uint8_t LESC_IND = 3;
    static constexpr uint8_t LTILT_IND = 4;
    static constexpr uint8_t LALR_IND = 5;
    static constexpr uint8_t ELE_IND = 6;
    static constexpr uint8_t NUM_DEVICES = 7;

    // time between writing output pins high
    static constexpr uint8_t PULSE_PADDING = 20; // microseconds

private:
    pwmDevice* devices[NUM_DEVICES];
    pwmDevice* sorted_devices[NUM_DEVICES];
};

#endif // PWM_DRIVER_H
