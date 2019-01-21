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
  PenguinJoystick p_joy1;

  frc::Joystick leftJoystick = frc::Joystick(usb0);
  frc::Joystick rightJoystick = frc::Joystick(usb1);

  // # ONBOARD #
  // SENSORS
  // Lift stage mag sensors
  DIO* liftBottom;
  DIO* liftMid;
  DIO* liftTop;
  
  // Line following sensors
  DIO* lineSensorLeft;
  DIO* lineSensorMid;
  DIO* lineSensorRight;

  Lidar* leftLidar = new Lidar(LEFT_LIDAR_NAVX__RIGHT_LIDAR_RIO);
  Lidar* rightLidar = new Lidar(!LEFT_LIDAR_NAVX__RIGHT_LIDAR_RIO /* maybe need to put in address */);
  int lidarDist;

  frc::Ultrasonic* leftUltrasonic = new frc::Ultrasonic(LEFT_ULTRASONIC_PING_CHANNEL, LEFT_ULTRASONIC_ECHO_CHANNEL);
  frc::Ultrasonic* rightUltrasonic = new frc::Ultrasonic(RIGHT_ULTRASONIC_PING_CHANNEL, RIGHT_ULTRASONIC_ECHO_CHANNEL);

  DIO* leftDioUltrasonic;
  DIO* rightDioUltrasonic;
  
  frc::SerialPort* serialUltrasonic = new frc::SerialPort(9600);
  frc::AnalogInput* analogUltrasonic = new frc::AnalogInput(0);


  int leftLidarDistance;
  int rightLidarDistance;
  int leftUltrasonicDistance;
  int rightUltrasonicDistance;

  // MOTOR CONTROLLERS
  // Talons
  WPI_TalonSRX l1{LEFT_1_CAN_ADDRESS};
  WPI_TalonSRX l2{LEFT_2_CAN_ADDRESS};
  WPI_TalonSRX r1{RIGHT_1_CAN_ADDRESS};
  WPI_TalonSRX r2{RIGHT_2_CAN_ADDRESS}; 

  frc::Spark intakeMotor{INTAKE_MOTOR_PWM_PORT};
  WPI_TalonSRX elevatorMotor{ELEVATOR_MOTOR_CAN_ADDRESS};

  // WPI_TalonSRX test_wpi_talon{0};

  frc::DifferentialDrive drive{l1, r1};

  // OTHER

  AHRS* ahrs = new AHRS(I2C::Port::kMXP);

  frc::Compressor compressor{pcm0};
  frc::DoubleSolenoid driveGearboxes{pcm0, pch0, pch1};
  frc::DoubleSolenoid intakePiston{pcm0, pch2, pch3};
  frc::DoubleSolenoid ballPusher{pcm0, pch4, pch5};
  frc::DoubleSolenoid hatchPusher{pcm0, pch6, pch7};


  //frc::Encoder leftEnc{dio3, dio2}, rightEnc{dio1, dio2}; // might need to switch

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
  void ToggleGear(bool btn, frc::DoubleSolenoid& solenoid);

  void HandleJoysticks();

  void GetDistances();
  void Approach();
  void LidarInit();

  void DriveLeft(double amount);
  void DriveRight(double amount);
  void DriveBoth(double amount);
  void Move(); //TODO: better name



  // Utils:
  double calculateDampenedJoystick(double rawAxisValue);

  void Testing();


  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
