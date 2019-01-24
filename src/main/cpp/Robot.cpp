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
  liftTop = new frc::DigitalInput(DIO_ELEVATOR_TOP);
  liftMid = new frc::DigitalInput(DIO_ELEVATOR_MID);
  liftBottom = new frc::DigitalInput(DIO_ELEVATOR_BOTTOM);

  lineSensorMid = new frc::DigitalInput(dio4);

  analogUltrasonic->InitAccumulator();
  analogUltrasonic->ResetAccumulator();

  serialUltrasonic->EnableTermination((char)31);
  serialUltrasonic->Reset();

  compressor.SetClosedLoopControl(true);
  ShiftGears(Direction::down, driveGearboxes);
  // intakeArm.Set(frc::DoubleSolenoid::kReverse);
  // ballPusher.Set(frc::DoubleSolenoid::kReverse);    // IS THIS RIGHT? (TODO)
  // hatchPusher.Set(frc::DoubleSolenoid::kReverse);

  TalonInit();
  elevatorSparkMotor.SetInverted(false); // TODO
  intakeMotor.SetInverted(false); // TODO
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  Testing();
  GetDistances();

  printf("left dist: %i, right dist: %i", leftLidarDistance, rightLidarDistance);


}

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
  { // Built-in auto code
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

}

void Robot::AutonomousPeriodic() {
  { // Built in auto code
    if (m_autoSelected == kAutoNameCustom) {
      // Custom Auto goes here
    } else {
      // Default Auto goes here
    }
  }

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  HandleButtons();
  Move();
  RunElevator(); 
/* frc::SmartDashboard::PutNumber("rawAxis0", rightJoystick.GetRawAxis(0));
  frc::SmartDashboard::PutNumber("rawAxis1", rightJoystick.GetRawAxis(1));
  frc::SmartDashboard::PutNumber("rawAxis2", rightJoystick.GetRawAxis(2));
  frc::SmartDashboard::PutNumber("rawAxis3", rightJoystick.GetRawAxis(3));
  frc::SmartDashboard::PutNumber("rawAxis4", rightJoystick.GetRawAxis(4));
  frc::SmartDashboard::PutNumber("rawAxis5", rightJoystick.GetRawAxis(5)); 
*/
}

void Robot::TestPeriodic() {
//   Testing();
}

//--------------------------

void Robot::TalonInit() {
  l1.SetInverted(true);
  l2.SetInverted(true);
  r1.SetInverted(false); 
  r2.SetInverted(false);

  l2.Follow(l1);
  r2.Follow(r1);

  l1.SetNeutralMode(NeutralMode::Brake);
  // l2.SetNeutralMode(NeutralMode::Coast);
  r1.SetNeutralMode(NeutralMode::Brake);
  // r2.SetNeutralMode(NeutralMode::Coast); 


  // r1.SetSafety
}

void Robot::DriveLeft(double amount) {
  l1.Set(ControlMode::PercentOutput, amount);
}
void Robot::DriveRight(double amount) {
  r1.Set(ControlMode::PercentOutput, amount);
}
void Robot::DriveBoth(double amount) {
  DriveLeft (amount);
  DriveRight(amount);
}
void Robot::Move() {
  double left = leftJoystick.GetRawAxis(1);
  double right = rightJoystick.GetRawAxis(1);
  double both;

  if(fabs(left - right) < 0.15) {
    both = (left + right) / 2;
    DriveBoth (calculateDampenedJoystick(both ));
  } else {
    DriveLeft (calculateDampenedJoystick(left ));
    DriveRight(calculateDampenedJoystick(right));
  }
}

void Robot::RunElevator() {
  double val = gamerJoystick.GetRawAxis(3); // right joy, up/down
  elevatorSparkMotor.Set(val * 0.2);
}

void Robot::ShiftGears(Robot::Direction dir, frc::DoubleSolenoid& solenoid) {
  frc::DoubleSolenoid::Value state;
	if (dir == Direction::up /* or Direction::forward? reverse should prob be down/back. */) {
		state = frc::DoubleSolenoid::kReverse;
	} else if (dir == Direction::down) {
		state = frc::DoubleSolenoid::kForward;
	}
  solenoid.Set(state);
}

void Robot::ShiftGears(frc::DoubleSolenoid::Value state, frc::DoubleSolenoid& solenoid) {
  solenoid.Set(state);
}

void Robot::ShiftGears(bool downBtn, bool upBtn, frc::DoubleSolenoid& solenoid) {
  if (upBtn) {
		ShiftGears(Direction::up, solenoid);
	}
	if (downBtn) {
		ShiftGears(Direction::down, solenoid);
  }
}

void Robot::ToggleSolenoid(bool btn, frc::DoubleSolenoid& solenoid) {
  if (btn) {
    ToggleSolenoid(solenoid);
  }

/* old code 
  // if(btn) {
  //   if ( currentGear == Direction::down) {
  //     currentGear = Direction::up;
  //   } else if( currentGear == Direction::up) {
  //     currentGear = Direction::down;
  //   }
  // }

  // ShiftGears(currentGear, driveGearboxes);
*/
}

void Robot::ToggleSolenoid(frc::DoubleSolenoid& solenoid) {
  frc::DoubleSolenoid::Value state = solenoid.Get();
  state = reverseStates[state];
  ShiftGears(state, solenoid);
}

void Robot::HandleButtons() {
  ToggleSolenoid(rightJoystick.GetRawButtonPressed(2), driveGearboxes);
  ToggleSolenoid(/* rightJoystick.GetRawButtonPressed(2) */ false, intakeArm);
  ToggleSolenoid(/* rightJoystick.GetRawButtonPressed(2) */ false, ballPusher);
  ToggleSolenoid(/* rightJoystick.GetRawButtonPressed(2) */ false, hatchPusher);


}

void Robot::GetDistances() {
  leftLidarDistance = leftLidar->AquireDistance();
  rightLidarDistance = rightLidar->AquireDistance();  
  // leftUltrasonicDistance = leftUltrasonic->GetRangeInches();
  // rightUltrasonicDistance = rightUltrasonic->GetRangeInches();

  frc::SmartDashboard::PutNumber("leftLidarDistance", leftLidarDistance);
  frc::SmartDashboard::PutNumber("rightLidarDistance", rightLidarDistance);
  // frc::SmartDashboard::PutNumber("leftUltrasonicDistance", leftUltrasonicDistance);
  // frc::SmartDashboard::PutNumber("rightUltrasonicDistance", rightUltrasonicDistance);

  int gotValue = analogUltrasonic->GetValue();
  int gotAverageValue = analogUltrasonic->GetAverageValue();
  // double gotVoltage = analogUltrasonic->GetVoltage();
  // double gotAverageVoltage = analogUltrasonic->GetAverageVoltage();

  


  frc::SmartDashboard::PutNumber("gotValue", gotValue);
  frc::SmartDashboard::PutNumber("gotAverageValue", gotAverageValue);
  // frc::SmartDashboard::PutNumber("gotVoltage", gotVoltage);
  // frc::SmartDashboard::PutNumber("gotAverageVoltage", gotAverageVoltage);

  // int bytesReceived = serialUltrasonic->GetBytesReceived();
  // char buffer[bytesReceived];
  // int readBytes = serialUltrasonic->Read(buffer, bytesReceived);

  // frc::SmartDashboard::PutNumber("bytesReceived", bytesReceived);
  // frc::SmartDashboard::PutRaw("buffer", buffer);
  // frc::SmartDashboard::PutNumber("readBytes", readBytes);

}
  

void Approach(double& left, double& right) {
  // double kP = 1.0;
  // double kI = 0.0;
  // double kD = 0.0;

   
  // if(currentState == State::LINING_UP) {
  //   double leftGreater = left - right;
  //   if(fabs(leftGreater) > 1 /* Parallel tolerance */) {
  //     if (leftGreater > 0) { // left further away
  //       // send more to left
  //       l1.Set(ControlMode::PercentOutput,  leftGreater);
  //       r1.Set(ControlMode::PercentOutput, -leftGreater);
  //     } else {
  //       l1.Set(ControlMode::PercentOutput, -leftGreater);
  //       r1.Set(ControlMode::PercentOutput,  leftGreater);
  //     }
  //   }
  // }
}

void LidarInit() {
  
}


double Robot::calculateDampenedJoystick(double rawAxisValue) {
  double dampening;
  if(fabs(rawAxisValue) <= 0.3) {
      dampening = 0.0;
  } else if(fabs(rawAxisValue) < 0.7) {
      dampening = 0.7;
  } else {
      dampening = 0.9;
  }
  return dampening * rawAxisValue;
}

void Robot::Testing() {
  // bool gotTopButton = leftJoystick.GetTop(); 
  // bool gotTriggerButton = leftJoystick.GetTrigger();
  // frc::SmartDashboard::PutBoolean("topButtonLeft", gotTopButton);
  // frc::SmartDashboard::PutBoolean("TriggerButtonLeft", gotTriggerButton);
  
  // frc::SmartDashboard::PutBoolean("should be shifting", \
                                  leftJoystick.GetRawButton(6) || leftJoystick.GetRawButton(4));

  // frc::SmartDashboard::PutBoolean("line sensor", lineSensorMid->Get());
  
}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif