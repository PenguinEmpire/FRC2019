/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>



void Robot::RobotInit() {
  // Auto Mode Chooser
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Object Initialization
  liftTop = new frc::DigitalInput(dio0);
  liftMid = new frc::DigitalInput(dio1);
  liftBottom = new frc::DigitalInput(dio2);

  
  compressor.SetClosedLoopControl(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  bool gotTopButton = joy1.GetTop();//frc::Joystick::ButtonType::kTopButton);
  bool gotTriggerButton = joy1.GetTrigger();//frc::Joystick::ButtonType::kTriggerButton);
  frc::SmartDashboard::PutBoolean("button", gotTopButton);
  frc::SmartDashboard::PutBoolean("button", gotTriggerButton);

  l1.Set(ControlMode::PercentOutput, joy1.GetRawAxis(1));
  l2.Set(ControlMode::PercentOutput, joy1.GetRawAxis(1));

  r1.Set(ControlMode::PercentOutput, joy2.GetRawAxis(1));
  r2.Set(ControlMode::PercentOutput, joy2.GetRawAxis(1));
}

void Robot::TestPeriodic() {}

void Robot::ShiftGears(Robot::Direction dir) {
  frc::DoubleSolenoid::Value state;
	if (dir == Robot::Direction::up) {
		state = frc::DoubleSolenoid::kForward;
	} else {
		state = frc::DoubleSolenoid::kReverse;
	}

  driveGearboxes.Set(state);
}

void Robot::ShiftGears(bool upBtn, bool downBtn) {
  if (upBtn) {
		ShiftGears(up);
	}
	if (downBtn) {
		ShiftGears(down);
  }
}





















#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
