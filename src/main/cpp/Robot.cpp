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

  //-----------------------------------------------------------------------------------

  //Object Initialization
  liftTop = new frc::DigitalInput(dio0);
  liftMid = new frc::DigitalInput(dio1);
  liftBottom = new frc::DigitalInput(dio2);

  compressor.SetClosedLoopControl(true);
  ShiftGears(currentGear);

  TalonInit();
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

//-----------------------------


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
  Testing();

  HandleJoysticks();
  Move();

/* frc::SmartDashboard::PutNumber("rawAxis0", rightJoystick.GetRawAxis(0));
  frc::SmartDashboard::PutNumber("rawAxis1", rightJoystick.GetRawAxis(1));
  frc::SmartDashboard::PutNumber("rawAxis2", rightJoystick.GetRawAxis(2));
  frc::SmartDashboard::PutNumber("rawAxis3", rightJoystick.GetRawAxis(3));
  frc::SmartDashboard::PutNumber("rawAxis4", rightJoystick.GetRawAxis(4));
  frc::SmartDashboard::PutNumber("rawAxis5", rightJoystick.GetRawAxis(5)); 
*/
}

void Robot::TestPeriodic() {
  Testing();
}

//--------------------------

void Robot::TalonInit() {
  l1.SetInverted(true);
  l2.SetInverted(true);
  r1.SetInverted(false); 
  r2.SetInverted(false);


  l1.SetNeutralMode(NeutralMode::Brake);
  // l2.SetNeutralMode(NeutralMode::Coast);
  r1.SetNeutralMode(NeutralMode::Brake);
  // r2.SetNeutralMode(NeutralMode::Coast);

  l2.Follow(l1);
  r2.Follow(l2);
}

void Robot::DriveLeft(double amount) {
  l1.Set(ControlMode::PercentOutput, amount);
  // l2.Follow(l1);
}
void Robot::DriveRight(double amount) {
  r1.Set(ControlMode::PercentOutput, amount);
  // r2.Follow(r1);
}
void Robot::DriveBoth(double amount) {
  DriveLeft(amount);
  DriveRight(amount);
}
void Robot::Move() {
  double left = leftJoystick.GetRawAxis(1);
  double right = rightJoystick.GetRawAxis(1);
  double both;

  if(fabs(left - right) < 0.1) {
    both = (left + right) / 2;
    DriveBoth (calculateDampenedJoystick(both ));
  } else {
    DriveLeft (calculateDampenedJoystick(left ));
    DriveRight(calculateDampenedJoystick(right));
  }
}

void Robot::ShiftGears(Robot::Direction dir) {
  frc::DoubleSolenoid::Value state;
	if (dir == Direction::up) {
		state = frc::DoubleSolenoid::kReverse;
	} else {
		state = frc::DoubleSolenoid::kForward;
	}

  driveGearboxes.Set(state);
}

void Robot::ShiftGears(bool downBtn, bool upBtn) {
  if (upBtn) {
		ShiftGears(Direction::up);
	}
	if (downBtn) {
		ShiftGears(Direction::down);
  }
}

void Robot::ToggleGear(bool btn) {
  if(btn) {
    if ( currentGear == Direction::down ) {
      currentGear = Direction::up;
    } else if( currentGear == Direction::up ) {
      currentGear = Direction::down;
    }
  }

  ShiftGears(currentGear);
}

void Robot::HandleJoysticks() {
  // ShiftGears(leftJoystick.GetRawButton(6), leftJoystick.GetRawButton(4));
  ToggleGear(rightJoystick.GetRawButtonPressed(2));

}

double Robot::calculateDampenedJoystick(double rawAxisValue) {
  double dampening;
  if(fabs(rawAxisValue) <= 0.3) {
      dampening = 0.0;
  } else if(fabs(rawAxisValue) < 0.7) {
      dampening = 1.0;
  } else {
      dampening = 0.9;
  }
  return dampening * rawAxisValue;
}

void Robot::Testing() {
  bool gotTopButton = leftJoystick.GetTop(); //frc::Joystick::ButtonType::kTopButton);
  bool gotTriggerButton = leftJoystick.GetTrigger(); //frc::Joystick::ButtonType::kTriggerButton);
  frc::SmartDashboard::PutBoolean("topButtonLeft", gotTopButton);
  frc::SmartDashboard::PutBoolean("TriggerButtonLeft", gotTriggerButton);
  //
  frc::SmartDashboard::PutBoolean("should be shifting", \
                                  leftJoystick.GetRawButton(6) || leftJoystick.GetRawButton(4));
  
}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif