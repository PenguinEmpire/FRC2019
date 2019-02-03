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

constexpr int LEFT_1_CAN_ADDRESS =  3;
constexpr int LEFT_2_CAN_ADDRESS =  0;
constexpr int RIGHT_1_CAN_ADDRESS = 2;
constexpr int RIGHT_2_CAN_ADDRESS = 1;
 
constexpr int INTAKE_MOTOR_PWM_PORT = pwm0;
constexpr int ELEVATOR_MOTOR_CAN_ADDRESS = can4;
constexpr int ELEVATOR_SPARK_PWM = pwm1;


constexpr int DIO_ELEVATOR_TOP = 7;
constexpr int DIO_ELEVATOR_MID = 8; // TODO: assign ports
constexpr int DIO_ELEVATOR_BOTTOM = 9;

/** # where the pneumatics are plugged in
 * driveGearboxes{pcm0, pch0, pch1};
 * intakePiston{pcm0, pch2, pch3};
 * ballPusher{pcm0, pch4, pch5};
 * driveGearboxes{pcm0, pch6, pch7}; 
*/

constexpr int LEFT_ULTRASONIC_PING_CHANNEL = 2;
constexpr int LEFT_ULTRASONIC_ECHO_CHANNEL = 3;
constexpr int RIGHT_ULTRASONIC_PING_CHANNEL = 4;
constexpr int RIGHT_ULTRASONIC_ECHO_CHANNEL = 5;

constexpr bool LEFT_LIDAR_NAVX__RIGHT_LIDAR_RIO = true;
constexpr frc::I2C::Port LEFT_LIDAR_PORT = frc::I2C::kOnboard;
constexpr frc::I2C::Port RIGHT_LIDAR_PORT = frc::I2C::kMXP;
