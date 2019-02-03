/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <unordered_map>

#include <frc/TimedRobot.h> 
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/WPILib.h>

#include "PenguinConstants.h"
#include "PenguinJoystick.h"
#include "Lidar.h"

#include "ctre/Phoenix.h"
#include "AHRS.h"
// #include "pathfinder.h"

using std::unordered_map;
typedef frc::DigitalInput DIO;

class Robot : public frc::TimedRobot {
 public:

  enum Direction {
    up, down, left, right, backward, forward
  } currentGear = down;

  unordered_map<frc::DoubleSolenoid::Value, frc::DoubleSolenoid::Value> reverseStates = { \
      {frc::DoubleSolenoid::kReverse, frc::DoubleSolenoid::kForward}, \
      {frc::DoubleSolenoid::kForward, frc::DoubleSolenoid::kReverse}, \
  };

  enum State {
      UNINITIALIZED,
      LINING_UP,
  } currentState = UNINITIALIZED;

/*   struct pneumatic {
    frc::DoubleSolenoid solenoid;
    Robot::Direction currentDir;
    
    pneumatic(int pcm, int pch_1, int pch_2, Direction startDir) {
      solenoid = frc::DoubleSolenoid::DoubleSolenoid(pcm, pch_1, pch_2);
      currentDir = startDir;
    }
    ~pneumatic() = delete;
  };
*/

  // # OFFBOARD #
  frc::Joystick leftJoystick = frc::Joystick(usb0);
  frc::Joystick rightJoystick = frc::Joystick(usb1);
  frc::Joystick gamerJoystick = frc::Joystick(usb2);

  // # ONBOARD #
  // SENSORS
  // Lift stage mag sensors
  DIO* liftBottom;
  DIO* liftMid;
  DIO* liftTop;
  
  // Line following sensors
  // DIO* lineSensorLeft;
  // DIO* lineSensorMid;
  // DIO* lineSensorRight;
  // frc::AnalogInput* lineSensorLeft = new frc::AnalogInput(0);
  // frc::AnalogInput* lineSensorMid = new frc::AnalogInput(1);
  // frc::AnalogInput* lineSensor2 = new frc::AnalogInput(2);
  // frc::AnalogInput* lineSensorRight = new frc::AnalogInput(3);
  // frc::AnalogInput* lineSensornavx4 = new frc::AnalogInput(4);

  Lidar* leftLidar = new Lidar(LEFT_LIDAR_PORT);
  Lidar* rightLidar = new Lidar(RIGHT_LIDAR_PORT /* maybe need to put in address */);

//  ## ULTRASONICS ##
  // frc::Ultrasonic* leftUltrasonic = new frc::Ultrasonic(LEFT_ULTRASONIC_PING_CHANNEL, LEFT_ULTRASONIC_ECHO_CHANNEL);
  // frc::Ultrasonic* rightUltrasonic = new frc::Ultrasonic(RIGHT_ULTRASONIC_PING_CHANNEL, RIGHT_ULTRASONIC_ECHO_CHANNEL);
  // DIO* leftDioUltrasonic;
  // DIO* rightDioUltrasonic;
  // frc::SerialPort* serialUltrasonic = new frc::SerialPort(9600);
  frc::AnalogInput* analogUltrasonicR = new frc::AnalogInput(1);
  frc::AnalogInput* analogUltrasonicL = new frc::AnalogInput(0);

  struct distance {
    int lidarL;
    int lidarR;
    int ultrasonicL;
    int ultrasonicR; 
  } distances;


  // MOTOR CONTROLLERS
  // Talons
  WPI_TalonSRX l1{LEFT_1_CAN_ADDRESS};
  WPI_TalonSRX l2{LEFT_2_CAN_ADDRESS};
  WPI_TalonSRX r1{RIGHT_1_CAN_ADDRESS};
  WPI_TalonSRX r2{RIGHT_2_CAN_ADDRESS}; 

  frc::Spark intakeMotor{INTAKE_MOTOR_PWM_PORT};
  WPI_TalonSRX elevatorTalonMotor{ELEVATOR_MOTOR_CAN_ADDRESS};
  frc::Spark elevatorSparkMotor{ELEVATOR_SPARK_PWM};


  // frc::DifferentialDrive drive{l1, r1};

  // OTHER

  AHRS* ahrs = new AHRS(I2C::Port::kMXP);

  frc::Compressor compressor{pcm0};
  frc::DoubleSolenoid driveGearboxes{pcm0, pch0, pch1};
  frc::DoubleSolenoid intakeArm{pcm0, pch2, pch3};
  frc::DoubleSolenoid ballPusher{pcm0, pch4, pch5};
  frc::DoubleSolenoid hatchPusher{pcm0, pch6, pch7};

  // frc::PIDController straighten = frc::PIDController();


  frc::Encoder leftEnc{dio3, dio2}, rightEnc{dio1, dio2}; // might need to switch

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void TalonInit();

  void ShiftGears(Robot::Direction dir, frc::DoubleSolenoid& solenoid);
  void ShiftGears(frc::DoubleSolenoid::Value state, frc::DoubleSolenoid& solenoid);
  void ShiftGears(bool upBtn, bool downBtn, frc::DoubleSolenoid& solenoid);
  void ToggleSolenoid(bool btn, frc::DoubleSolenoid& solenoid);
  void ToggleSolenoid(frc::DoubleSolenoid& Solenoid);

  void GetDistances();
  void Approach(int left, int right, int tolerance);
  void LidarInit();

  void DriveLeft(double amount);
  void DriveRight(double amount);
  void DriveBoth(double amount);
  void HandleJoysticks(); //TODO: better name
  void RunElevator();



  // Utils:
  double calculateDampenedJoystick(double rawAxisValue);

  void Testing();


  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
