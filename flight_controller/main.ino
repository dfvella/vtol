#include <Arduino.h>

#include "imu_driver.h"
#include "ppm_decoder.h"
#include "pwm_driver.h"
#include "control_map.h"
#include "pid_controller.h"

#include "flight_controller.h"

#include "serial_logger.h"

//#define CALIBRATE_ACCELEROMETER
#define ENABLE_SERVOS

// units: degrees
#define MAX_ROLL_ANGLE 60.0
#define MAX_PITCH_ANGLE 60.0

// units: degrees
#define AUTOCLIMB_TRIM 15 // was 20, 15

// units: degrees
#define PID_ROLL_TRIM 2
#define PID_PITCH_TRIM -1

#define BRAKE_NOTCH_1 100
#define BRAKE_NOTCH_2 400

// Pin assignments
#define PPM_PIN 3
#define RIGHT_MOTOR_PIN 4
#define RIGHT_TILT_PIN 5
#define RIGHT_ALR_PIN 6
#define LEFT_MOTOR_PIN 7
#define LEFT_TILT_PIN 8
#define LEFT_ALR_PIN 9
#define ELE_PIN 10
#define LED_PIN 13

unsigned long timer = 0;
const int LOOP_TIME = 5000; // microseconds

Imu imu{ LED_PIN };
ppmDecoder ppm;

pwmDevice right_motor{ RIGHT_MOTOR_PIN, RIGHT_MOTOR_MIN_PULSE, RIGHT_MOTOR_MAX_PULSE };
pwmDevice right_tilt{ RIGHT_TILT_PIN, RIGHT_TILT_MIN_PULSE, RIGHT_TILT_MAX_PULSE };
pwmDevice right_alr{ RIGHT_ALR_PIN, RIGHT_ALR_MIN_PULSE, RIGHT_ALR_MAX_PULSE };
pwmDevice left_motor{ LEFT_MOTOR_PIN, LEFT_MOTOR_MIN_PUSLE, LEFT_MOTOR_MAX_PULSE };
pwmDevice left_tilt{ LEFT_TILT_PIN, LEFT_TILT_MIN_PULSE, LEFT_TILT_MAX_PULSE };
pwmDevice left_alr{ LEFT_ALR_PIN, LEFT_ALR_MIN_PULSE, LEFT_ALR_MAX_PULSE };
pwmDevice elevator{ ELE_PIN, ELE_MIN_PULSE, ELE_MAX_PULSE };
pwmScheduler pwm_scheduler;

Flight_Controller flight_controller{ imu, ppm };

enum class Controller_State : uint8_t { PPMSYNC, PIDCALC, SERVOSET, SERVOWRITE };

void setup() 
{
    pinMode(LED_PIN, OUTPUT);

    #ifdef SERIAL_CONNECTION
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Serial connection");
    #endif

    #ifndef CALIBRATE_ACCELEROMETER
    imu.calibrate();
    #else
    imu.calibrate_accel();
    #endif

    assignPpmDecoderToPin(ppm, PPM_PIN);

    pwm_scheduler.add_device(pwmScheduler::RESC_IND, &right_motor);
    pwm_scheduler.add_device(pwmScheduler::RTILT_IND, &right_tilt);
    pwm_scheduler.add_device(pwmScheduler::RALR_IND, &right_alr);
    pwm_scheduler.add_device(pwmScheduler::LESC_IND, &left_motor);
    pwm_scheduler.add_device(pwmScheduler::LTILT_IND, &left_tilt);
    pwm_scheduler.add_device(pwmScheduler::LALR_IND, &left_alr);
    pwm_scheduler.add_device(pwmScheduler::ELE_IND, &elevator);

    digitalWrite(LED_PIN, LOW);
    timer = micros();
}

void loop() 
{
    static Controller_State state = Controller_State::PPMSYNC;

    static int16_t arl_out, ele_out, rud_out, brk_out;

    float roll_target, pitch_target;

    imu.run();

    switch (state)
    {
    case Controller_State::PPMSYNC:
        flight_controller.ppmsync();
        state = Controller_State::PIDCALC;
        break;

    case Controller_State::PIDCALC:
        roll_target = (float)Servo::CENTER - ppm.get(ppmDecoder::ARL);
        pitch_target = (float)Servo::CENTER - ppm.get(ppmDecoder::ELE);

        roll_target *= MAX_ROLL_ANGLE / 1000;
        pitch_target *= MAX_PITCH_ANGLE / 1000;

        if (fmode == PASSTHRU)
        {
            arl_out = ppm.get(ppmDecoder::ARL);
            ele_out = ppm.get(ppmDecoder::ELE);
        }
        else
        {
            uint16_t trim = 0;
            if (fmode == AUTOCLIMB)
                trim = AUTOCLIMB_TRIM;

            arl_out = roll_pid.calculate(imu.roll() - roll_target + PID_ROLL_TRIM);
            ele_out = pitch_pid.calculate(imu.pitch() - pitch_target + PID_PITCH_TRIM - trim);

            arl_out = Servo::limit(arl_out + Servo::CENTER);
            ele_out = Servo::limit(ele_out + Servo::CENTER);
        }
        rud_out = ppm.get(ppmDecoder::RUD);

        brk_out = 0;
        if (ppm.get(ppmDecoder::GER) < SWITCH_POS2)
            brk_out = BRAKE_NOTCH_1;
        if (ppm.get(ppmDecoder::GER) < SWITCH_POS1)
            brk_out = BRAKE_NOTCH_2;

        state = Controller_State::SERVOSET;
        break;

    case Controller_State::SERVOSET:
        servo[THR]->set(ppm.get(ppmDecoder::THR));

        servo[RTS]->set(Mix::right_top(arl_out, ele_out, rud_out, brk_out));
        servo[RBS]->set(Mix::right_bottom(arl_out, ele_out, rud_out, brk_out));
        servo[LTS]->set(Mix::left_top(arl_out, ele_out, rud_out, brk_out));
        servo[LBS]->set(Mix::left_bottom(arl_out, ele_out, rud_out, brk_out));

        state = Controller_State::SERVOWRITE;
        break;

    case Controller_State::SERVOWRITE:
        #ifdef ENABLE_SERVOS
        pwm_scheduler.write_all();
        #endif 

        state = Controller_State::PPMSYNC;
        break;
    }

    #ifdef PRINT_LOOP_TIME
    uint16_t loop_time = micros() - timer;
    #endif

    #ifdef DO_LOGGING
    print_log()
    #endif

    if (micros() - timer > LOOP_TIME) 
        digitalWrite(LED_PIN, HIGH);
    else
        digitalWrite(LED_PIN, LOW);

    while (micros() - timer < LOOP_TIME);
    timer = micros();
}
