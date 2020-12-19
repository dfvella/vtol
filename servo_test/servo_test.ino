#include <Arduino.h>

#include "pwm_driver.h"

//#define DO_PRINTS

#define PPM_PIN 3
#define RIGHT_MOTOR_PIN 4
#define RIGHT_TILT_PIN 5
#define RIGHT_ALR_PIN 6
#define LEFT_MOTOR_PIN 7
#define LEFT_TILT_PIN 8
#define LEFT_ALR_PIN 9
#define ELE_PIN 10
#define LED_PIN 13

pwmDevice right_motor{ RIGHT_MOTOR_PIN, RIGHT_MOTOR_MIN_PULSE, RIGHT_MOTOR_MAX_PULSE };
pwmDevice right_tilt{ RIGHT_TILT_PIN, RIGHT_TILT_MIN_PULSE, RIGHT_TILT_MAX_PULSE };
pwmDevice right_alr{ RIGHT_ALR_PIN, RIGHT_ALR_MIN_PULSE, RIGHT_ALR_MAX_PULSE };
pwmDevice left_motor{ LEFT_MOTOR_PIN, LEFT_MOTOR_MIN_PUSLE, LEFT_MOTOR_MAX_PULSE };
pwmDevice left_tilt{ LEFT_TILT_PIN, LEFT_TILT_MIN_PULSE, LEFT_TILT_MAX_PULSE };
pwmDevice left_alr{ LEFT_ALR_PIN, LEFT_ALR_MIN_PULSE, LEFT_ALR_MAX_PULSE };
pwmDevice elevator{ ELE_PIN, ELE_MIN_PULSE, ELE_MAX_PULSE };
pwmScheduler pwm_scheduler;

unsigned long timer = 0;
unsigned long counter = 0;

void setup()
{
    #ifdef DO_PRINTS
    Serial.begin(9600);
    #endif

    pwm_scheduler.add_device(pwmScheduler::RESC_IND, &right_motor);
    pwm_scheduler.add_device(pwmScheduler::RTILT_IND, &right_tilt);
    pwm_scheduler.add_device(pwmScheduler::RALR_IND, &right_alr);
    pwm_scheduler.add_device(pwmScheduler::LESC_IND, &left_motor);
    pwm_scheduler.add_device(pwmScheduler::LTILT_IND, &left_tilt);
    pwm_scheduler.add_device(pwmScheduler::LALR_IND, &left_alr);
    pwm_scheduler.add_device(pwmScheduler::ELE_IND, &elevator);

    timer = micros();
}

void loop()
{
    pwm_scheduler.set_signal(pwmScheduler::RESC_IND, RIGHT_MOTOR_MIN_PULSE);
    pwm_scheduler.set_signal(pwmScheduler::LESC_IND, LEFT_MOTOR_MIN_PUSLE);

    static uint16_t signal = 1000;

    if (counter >= 500)
        counter = 0;

    if (counter < 250)
        signal += 5;
    else
        signal -= 5;
    
    
    pwm_scheduler.set_signal(pwmScheduler::RTILT_IND, signal);
    pwm_scheduler.set_signal(pwmScheduler::RALR_IND, signal);
    pwm_scheduler.set_signal(pwmScheduler::LTILT_IND, signal);
    pwm_scheduler.set_signal(pwmScheduler::LALR_IND, signal);
    pwm_scheduler.set_signal(pwmScheduler::ELE_IND, signal);

    pwm_scheduler.write_all();

    #ifdef DO_PRINTS
    Serial.print(right_motor.get_signal());
    Serial.print(' ');
    Serial.print(right_tilt.get_signal());
    Serial.print(' ');
    Serial.print(right_alr.get_signal());
    Serial.print(' ');
    Serial.print(counter);
    Serial.print('\n');
    #endif

    ++counter;

    while (micros() - timer < 20000);
    timer = micros();
}
