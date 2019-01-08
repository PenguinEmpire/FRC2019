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
#include <WPILib.h>

#include "penguinports.h"
#include "PenguinJoystick.h"

#include <ctre/Phoenix.h>

class Robot : public frc::TimedRobot {
 public: 
 //Lift stage mag sensors
  frc::DigitalInput* liftBottom;
  frc::DigitalInput* liftMid;
  frc::DigitalInput* liftTop;

//Motor controllers
  //TODO - Add TalonSRX instances
  TalonSRX srx = {0};

  PenguinJoystick p_joy1;

  frc::Joystick joy1;


  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
