
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/WPILib.h>

#include "PenguinConstants.h"
#include "PenguinJoystick.h"

#include "ctre/Phoenix.h"
#include "AHRS.h"

typedef frc::DigitalInput DIO;

class Robot : public frc::TimedRobot {
 public:

  enum Direction {
    up, down, left, right, backward, forward
  };

  Direction currentGear = Direction::down;

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

  // MOTOR CONTROLLERS
  // Talons
  TalonSRX l1{LEFT_1_CAN_ADDRESS};
  TalonSRX l2{LEFT_2_CAN_ADDRESS};
  TalonSRX r1{RIGHT_1_CAN_ADDRESS};
  TalonSRX r2{RIGHT_2_CAN_ADDRESS}; 

  // OTHER

  AHRS* ahrs = new AHRS(I2C::Port::kMXP);

  frc::Compressor compressor{pcm0};
  frc::DoubleSolenoid driveGearboxes{pcm0, pch0, pch1};

  //frc::Encoder leftEnc{dio3, dio2}, rightEnc{dio1, dio2}; // might need to switch

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void TalonInit();

  void ShiftGears(Robot::Direction dir);
  void ShiftGears(bool upBtn, bool downBtn);
  void ToggleGear(bool btn);

  void HandleJoysticks();

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
