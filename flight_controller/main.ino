#include <Arduino.h>

#include "ppm_decoder.h"
#include "pwm_driver.h"
#include "flight_controller.h"

#include "serial_logger.h"

#define ENABLE_SERVOS

// Pin assignments
#define PPM_PIN 3
#define RIGHT_MOTOR_PIN 4
#define RIGHT_TILT_PIN 5
#define RIGHT_ALR_PIN 6
#define LEFT_MOTOR_PIN 7
#define LEFT_TILT_PIN 8
#define LEFT_ALR_PIN 9
#define ELE_PIN 10

unsigned long timer = 0;
const int LOOP_TIME = 5000; // microseconds

ppmDecoder ppm;

pwmDevice right_motor{ RIGHT_MOTOR_PIN, RIGHT_MOTOR_MIN_PULSE, RIGHT_MOTOR_MAX_PULSE };
pwmDevice right_tilt{ RIGHT_TILT_PIN, RIGHT_TILT_MIN_PULSE, RIGHT_TILT_MAX_PULSE };
pwmDevice right_alr{ RIGHT_ALR_PIN, RIGHT_ALR_MIN_PULSE, RIGHT_ALR_MAX_PULSE };
pwmDevice left_motor{ LEFT_MOTOR_PIN, LEFT_MOTOR_MIN_PULSE, LEFT_MOTOR_MAX_PULSE };
pwmDevice left_tilt{ LEFT_TILT_PIN, LEFT_TILT_MIN_PULSE, LEFT_TILT_MAX_PULSE };
pwmDevice left_alr{ LEFT_ALR_PIN, LEFT_ALR_MIN_PULSE, LEFT_ALR_MAX_PULSE };
pwmDevice elevator{ ELE_PIN, ELE_MIN_PULSE, ELE_MAX_PULSE };
pwmScheduler pwm_scheduler;

Flight_Controller flight_controller;

enum class Controller_State : uint8_t { PPMSYNC, PIDCALC, SERVOSET, SERVOWRITE };

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);

    #ifdef SERIAL_CONNECTION
    Serial.begin(9600);
    Serial.println("Serial connection");
    #endif

    flight_controller.begin();

    assignPpmDecoderToPin(ppm, PPM_PIN);

    pwm_scheduler.add_device(pwmScheduler::RESC_IND, &right_motor);
    pwm_scheduler.add_device(pwmScheduler::RTILT_IND, &right_tilt);
    pwm_scheduler.add_device(pwmScheduler::RALR_IND, &right_alr);
    pwm_scheduler.add_device(pwmScheduler::LESC_IND, &left_motor);
    pwm_scheduler.add_device(pwmScheduler::LTILT_IND, &left_tilt);
    pwm_scheduler.add_device(pwmScheduler::LALR_IND, &left_alr);
    pwm_scheduler.add_device(pwmScheduler::ELE_IND, &elevator);

    digitalWrite(LED_BUILTIN, LOW);

    timer = micros();
}

void loop() 
{
    static Controller_State state = Controller_State::PPMSYNC;

    static Flight_Controller::Output controller_output;

    flight_controller.run();

    switch (state)
    {
    case Controller_State::PPMSYNC:
        ppm.sync();

        state = Controller_State::PIDCALC;
        break;

    case Controller_State::PIDCALC:
        {
            Flight_Controller::Input input = {
                ppm.get(ppmDecoder::THR),
                ppm.get(ppmDecoder::ARL),
                ppm.get(ppmDecoder::ELE),
                ppm.get(ppmDecoder::RUD),
                ppm.get(ppmDecoder::GER),
                ppm.get(ppmDecoder::AUX)
            };
            flight_controller.calculate_outputs(input, controller_output);
        }

        state = Controller_State::SERVOSET;
        break;

    case Controller_State::SERVOSET:
        pwm_scheduler.set_signal(pwmScheduler::RESC_IND, controller_output.right_motor);
        pwm_scheduler.set_signal(pwmScheduler::RTILT_IND, controller_output.right_tilt);
        pwm_scheduler.set_signal(pwmScheduler::RALR_IND, controller_output.right_alr);
        pwm_scheduler.set_signal(pwmScheduler::LESC_IND, controller_output.left_motor);
        pwm_scheduler.set_signal(pwmScheduler::LTILT_IND, controller_output.left_tilt);
        pwm_scheduler.set_signal(pwmScheduler::LALR_IND, controller_output.left_alr);
        pwm_scheduler.set_signal(pwmScheduler::ELE_IND, controller_output.elevator);

        state = Controller_State::SERVOWRITE;
        break;

    case Controller_State::SERVOWRITE:
        #ifdef ENABLE_SERVOS
        pwm_scheduler.write_all();
        #endif 

        state = Controller_State::PPMSYNC;
        break;
    }

    #ifdef DO_LOGGING
    print_log()
    #endif

    if (micros() - timer > LOOP_TIME) 
        digitalWrite(LED_BUILTIN, HIGH);
    else
        digitalWrite(LED_BUILTIN, LOW);

    while (micros() - timer < LOOP_TIME);
    timer = micros();
}
