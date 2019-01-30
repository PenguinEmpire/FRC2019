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
  
  // Lidar* leftLidar = new Lidar(LEFT_LIDAR_NAVX__RIGHT_LIDAR_RIO);
  // Lidar* rightLidar = new Lidar(!LEFT_LIDAR_NAVX__RIGHT_LIDAR_RIO /* maybe need to put in address */);
  // int lidarDist;

  // frc::AnalogInput* analogUltrasonicL = new frc::AnalogInput(0);
  // frc::AnalogInput* analogUltrasonic = new frc::AnalogInput(1);

  // int leftLidarDistance;
  // int rightLidarDistance;
  // int leftUltrasonicDistance;
  // int rightUltrasonicDistance;

  // MOTOR CONTROLLERS
  // Talons
  frc::Spark l1{LEFT1_PWM_PORT };
  frc::Spark l2{LEFT2_PWM_PORT };
  frc::Spark r1{RIGHT1_PWM_PORT};
  frc::Spark r2{RIGHT2_PWM_PORT}; 

  frc::Spark intakeMotor{INTAKE_MOTOR_PWM_PORT};
  frc::Spark elevatorMotor{ELEVATOR_SPARK_PWM_PORT};

  // OTHER

  // AHRS* ahrs = new AHRS(I2C::Port::kMXP);

  frc::Compressor compressor{pcm0};
  frc::DoubleSolenoid driveGearboxes{pcm0, pch0, pch1};
  frc::DoubleSolenoid intakeArm{pcm0, pch6, pch7};
  frc::DoubleSolenoid ballPusher{pcm0, pch4, pch5};
  frc::DoubleSolenoid hatchPusher{pcm0, pch2, pch3};

  frc::Encoder leftEnc{dio0, dio1}, rightEnc{dio2, dio3}; // might need to switch

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void SparkInvert();

  void ShiftGears(Robot::Direction dir, frc::DoubleSolenoid& solenoid);
  void ShiftGears(frc::DoubleSolenoid::Value state, frc::DoubleSolenoid& solenoid);
  void ShiftGears(bool upBtn, bool downBtn, frc::DoubleSolenoid& solenoid);
  void ToggleSolenoid(bool btn, frc::DoubleSolenoid& solenoid);
  void ToggleSolenoid(frc::DoubleSolenoid& Solenoid);

  void HandleButtons();

  void GetDistances();
  void Approach();
  void LidarInit();

  void DriveLeft(double amount);
  void DriveRight(double amount);
  void DriveBoth(double amount);
  void Move(); //TODO: better name
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
