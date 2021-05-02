#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

#define SERIAL_BAUD_RATE 9600

// uncomment one of the following:
//#define PRINT_ROLL_ANGLE
//#define PRINT_PITCH_ANGLE
//#define PRINT_YAW_ANGLE
// or uncomment one or more of the following:
//#define PRINT_LOOP_TIME
//#define PRINT_IMU_ANGLES
//#define PRINT_PPM_INPUTS
//#define PRINT_CONTROLLER_TARGETS
//#define PRINT_CONTROLLER_OUTPUTS
//#define PRINT_FLIGHT_MODE
//#define PRINT_CONTROL_MODE

// by default send data at 200Hz
// uncomment the following line to send at 50Hz
//#define SLOW_LOGGING

#ifdef PRINT_LOOP_TIME
    #define DO_LOGGING
    #define print_loop_time() \
        Serial.print(micros() - timer); \
        Serial.print(' ');
#else 
    #define print_loop_time()
#endif

#ifdef PRINT_ROLL_ANGLE
    #define DO_LOGGING
    #define print_roll_angle() \
        Serial.print(flight_controller.get_roll_angle());
#else
    #define print_roll_angle()
#endif

#ifdef PRINT_PITCH_ANGLE
    #define DO_LOGGING
    #define print_pitch_angle() \
        Serial.print(flight_controller.get_pitch_angle());
#else
    #define print_pitch_angle()
#endif

#ifdef PRINT_YAW_ANGLE
    #define DO_LOGGING
    #define print_yaw_angle() \
        Serial.print(flight_controller.get_yaw_angle());
#else
    #define print_yaw_angle()
#endif

#ifdef PRINT_IMU_ANGLES
    #define DO_LOGGING
    #define print_imu_angles() \
        Serial.print(flight_controller.get_roll_angle()); \
        Serial.print(' '); \
        Serial.print(flight_controller.get_pitch_angle()); \
        Serial.print(' '); \
        Serial.print(flight_controller.get_yaw_angle()); \
        Serial.print(' ');
#else 
    #define print_imu_angles() 
#endif

#ifdef PRINT_PPM_INPUTS
    #define DO_LOGGING
    #define print_ppm_inputs() \
        Serial.print(ppm.get(ppmDecoder::ARL)); \
        Serial.print(' '); \
        Serial.print(ppm.get(ppmDecoder::ELE)); \
        Serial.print(' '); \
        Serial.print(ppm.get(ppmDecoder::THR)); \
        Serial.print(' '); \
        Serial.print(ppm.get(ppmDecoder::RUD)); \
        Serial.print(' '); \
        Serial.print(ppm.get(ppmDecoder::GER)); \
        Serial.print(' '); \
        Serial.print(ppm.get(ppmDecoder::AUX)); \
        Serial.print(' ');
#else 
    #define print_ppm_inputs()
#endif

#ifdef PRINT_CONTROLLER_TARGETS
    #define DO_LOGGING
    #define print_controller_targets() \
        Serial.print(flight_controller.get_target_roll()); \
        Serial.print(' '); \
        Serial.print(flight_controller.get_target_pitch()); \
        Serial.print(' '); \
        Serial.print(flight_controller.get_target_yaw()); \
        Serial.print(' ');
#else 
    #define print_controller_targets()
#endif

#ifdef PRINT_CONTROLLER_OUTPUTS
    #define DO_LOGGING
    #define print_controller_outputs() \
        Serial.print(controller_output.right_motor); \
        Serial.print(' '); \
        Serial.print(controller_output.right_tilt); \
        Serial.print(' '); \
        Serial.print(controller_output.right_alr); \
        Serial.print(' '); \
        Serial.print(controller_output.left_motor); \
        Serial.print(' '); \
        Serial.print(controller_output.left_tilt); \
        Serial.print(' '); \
        Serial.print(controller_output.left_alr); \
        Serial.print(' '); \
        Serial.print(controller_output.elevator); \
        Serial.print(' ');
#else 
    #define print_controller_outputs()
#endif

#ifdef PRINT_FLIGHT_MODE
    #define DO_LOGGING
    #define print_flight_mode() \
        Serial.print((uint8_t)flight_controller.get_flight_mode()); \
        Serial.print(' '); \
        Serial.print(flight_controller.get_transition_state()); \
        Serial.print(' ');
#else 
    #define print_flight_mode()
#endif

#ifdef PRINT_CONTROL_MODE
    #define DO_LOGGING
    #define print_control_mode() \
        Serial.print((uint8_t)flight_controller.get_control_mode()); \
        Serial.print(' ');
#else 
    #define print_control_mode()
#endif

#define print_log() \
    print_roll_angle() \
    print_pitch_angle() \
    print_yaw_angle() \
    print_loop_time() \
    print_imu_angles() \
    print_ppm_inputs() \
    print_controller_targets() \
    print_controller_outputs() \
    print_flight_mode() \
    print_control_mode() \
    Serial.println();

#ifdef DO_LOGGING
    #define SERIAL_CONNECTION
    #ifdef SLOW_LOGGING
        #define DO_LOGGING_50HZ
    #else
        #define DO_LOGGING_200HZ
    #endif
#endif


#endif // LOGGING_H
