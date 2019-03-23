#pragma once

#include "frc/I2C.h"

constexpr int dio0 = 0;
constexpr int dio1 = 1;
constexpr int dio2 = 2;
constexpr int dio3 = 3;
constexpr int dio4 = 4;
constexpr int dio5 = 5;
constexpr int dio6 = 6;     
constexpr int dio7 = 7;
constexpr int dio8 = 8;
constexpr int dio9 = 9;

constexpr int pch0 = 0;
constexpr int pch1 = 1;
constexpr int pch2 = 2;
constexpr int pch3 = 3;
constexpr int pch4 = 4;
constexpr int pch5 = 5;
constexpr int pch6 = 6;
constexpr int pch7 = 7;

constexpr int pcm0 = 0;

constexpr int pwm0 = 0;
constexpr int pwm1 = 1;
constexpr int pwm2 = 2;
constexpr int pwm3 = 3;
constexpr int pwm4 = 4;
constexpr int pwm5 = 5;
constexpr int pwm6 = 6;
constexpr int pwm7 = 7;

constexpr int usb0 = 0; 
constexpr int usb1 = 1;
constexpr int usb2 = 2;

constexpr int can0 = 0;
constexpr int can1 = 1;
constexpr int can2 = 2;
constexpr int can3 = 3;
constexpr int can4 = 4;
constexpr int can5 = 5;
constexpr int can6 = 6;
constexpr int can7 = 7;

constexpr float PULSE_IN = 0.16;

#define COMP_ROBOT                true // CHECK!
#define PRACTICE_TALON            true
#define LIDAR_EXIST               false 
#define ULTRA_EXIST               false
#define LIMELIGHT_EXIST           true
#define ELEVATOR_SENSOR_EXIST     false
#define PNEUMATIC_OBJECT          false

// false in comp ones
#define DO_PRINTF                 false  // false in comp, prob
#define PROFILING                 false
#define DO_DIAGNOSTIC             false
#define DO_EXTRA_IO               false  // also prob false
#define ALLOW_CALIBRATE_NAVX      false

#define DO_RUMBLE                 false

#define OPEN_LOOP_RAMP            false
#define LIMELIGHT_APPROACH        true
#define COVER_LAST_DIST_APPROACH  false

#define FANCY_NAVX                true
#define NAVX_BANGBANG             true

#define WALL_PROTECTION           false // probably don't change
#define ELEVATOR_DOWN_PROTECTION  true

// COMP ROBOT VALUES

#if COMP_ROBOT
    constexpr int LEFT_1_CAN_ADDRESS  =  0;
    constexpr int LEFT_2_CAN_ADDRESS  = 10;
    constexpr int RIGHT_1_CAN_ADDRESS =  2;
    constexpr int RIGHT_2_CAN_ADDRESS = 11;
#elif PRACTICE_TALON
    constexpr int LEFT_1_CAN_ADDRESS  =  3;
    constexpr int LEFT_2_CAN_ADDRESS  = 11;
    constexpr int RIGHT_1_CAN_ADDRESS =  6;
    constexpr int RIGHT_2_CAN_ADDRESS = 10;
#else
    constexpr int L1_SPARK_PWM = 3;
    constexpr int L2_SPARK_PWM = 4;
    constexpr int R1_SPARK_PWM = 1;
    constexpr int R2_SPARK_PWM = 2;
#endif

constexpr int INTAKE_MOTOR_PWM = 0;
constexpr int ELEVATOR_MOTOR_CAN_ADDRESS = 5;

#if ELEVATOR_SENSOR_EXIST
    constexpr int ELEVATOR_ZERO_HALL_DIO = 4;
#endif

#if ULTRA_EXIST
    constexpr int ULTRASONIC_R_ANALOG_IN = 1;
    constexpr int ULTRASONIC_L_ANALOG_IN = 0;
#endif

#if LIDAR_EXIST
    constexpr frc::I2C::Port LEFT_LIDAR_PORT = frc::I2C::kMXP;
    constexpr frc::I2C::Port RIGHT_LIDAR_PORT = frc::I2C::kOnboard;
#endif

//--

constexpr double DRIVE_OPENLOOP_RAMP = 0.5;

//--

constexpr double ELEVATOR_FEEDFORWARD = 0.2;
constexpr double ELEVATOR_DOWNSPEED = -0.6;

/** # where the pneumatics are plugged in
 * driveGearboxes{pcm0, pch0, pch1};
 * intakePiston{pcm0, pch2, pch3};
 * ballPusher{pcm0, pch4, pch5};
 * driveGearboxes{pcm0, pch6, pch7}; 
*/

/* DIO ultrasonics - deprecated
    constexpr int LEFT_ULTRASONIC_PING_CHANNEL = 2;
    constexpr int LEFT_ULTRASONIC_ECHO_CHANNEL = 3;
    constexpr int RIGHT_ULTRASONIC_PING_CHANNEL = 4;
    constexpr int RIGHT_ULTRASONIC_ECHO_CHANNEL = 5;
*/