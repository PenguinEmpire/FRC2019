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

  // lineSensorLeft = new frc::DigitalInput(7);  //  6);//10);
  // lineSensorMid = new frc::DigitalInput(8);  //  );//11);
  // lineSensorRight = new frc::DigitalInput(9);  //  (4);//12);

  analogUltrasonicR->InitAccumulator();
  analogUltrasonicR->ResetAccumulator();
  analogUltrasonicL->InitAccumulator();
  analogUltrasonicL->ResetAccumulator();
 
  // lineSensorLeft->InitAccumulator();
  // lineSensorLeft->ResetAccumulator();
  // lineSensorMid->InitAccumulator();
  // lineSensorMid->ResetAccumulator();
  // lineSensorRight->InitAccumulator();
  // lineSensorRight->ResetAccumulator();
  // lineSensor2->InitAccumulator();
  // lineSensor2->ResetAccumulator();
  // lineSensornavx4->InitAccumulator();
  // lineSensornavx4->ResetAccumulator();

  // serialUltrasonic->EnableTermination((char)31);
  // serialUltrasonic->Reset();

  leftEnc.SetDistancePerPulse(PULSE_IN);
  rightEnc.SetDistancePerPulse(PULSE_IN);

  leftEnc.Reset();
  rightEnc.Reset();

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

  leftEnc.Reset();
  rightEnc.Reset();


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
  HandleJoysticks();
  // Move();
  RunElevator();
}

void Robot::TestPeriodic() {}

//--------------------------

void Robot::TalonInit() {
  l1.SetInverted(true);
  l2.SetInverted(true);
  r1.SetInverted(false); 
  r2.SetInverted(false);

  // l2.Follow(l1);
  // r2.Follow(r1);

  l1.SetNeutralMode(NeutralMode::Brake);
  r1.SetNeutralMode(NeutralMode::Brake);

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
void Robot::HandleJoysticks() {
  double left = leftJoystick.GetRawAxis(1);
  double right = rightJoystick.GetRawAxis(1);
  double both;

  double toleranceScalar = ((leftJoystick.GetRawAxis(3) + 1) / 2);
  int ultraTol = 240 * toleranceScalar;
  int lidarTol =  80 * toleranceScalar;
  frc::SmartDashboard::PutNumber("ultraTol", ultraTol);
  frc::SmartDashboard::PutNumber("lidarTol", lidarTol);

  if (leftJoystick.GetRawButton(3)) {
    Align(distances.ultrasonicL, distances.ultrasonicR, ultraTol, DistanceType::ultrasonic);
    frc::SmartDashboard::PutBoolean("appr-ultrasonic", true);
  } else if (leftJoystick.GetRawButton(4)) {
    Align(distances.lidarL, distances.lidarR, lidarTol, DistanceType::lidar);
    frc::SmartDashboard::PutBoolean("appr-lidar", true);
  } else {
    frc::SmartDashboard::PutBoolean("appr-ultrasonic", false);
    frc::SmartDashboard::PutBoolean("appr-lidar", false);

    // Joystick Inputs
    if(fabs(left - right) < leftJoystick.GetRawAxis(3)) {
      both = (left + right) / 2;
      DriveBoth (calculateDampenedJoystick(both ));
    } else {
      DriveLeft (calculateDampenedJoystick(left ));
      DriveRight(calculateDampenedJoystick(right));
    }
  }

  ToggleSolenoid(rightJoystick.GetRawButtonPressed(2), driveGearboxes);
  ToggleSolenoid(/* rightJoystick.GetRawButtonPressed(2) */ false, intakeArm);
  ToggleSolenoid(/* rightJoystick.GetRawButtonPressed(2) */ false, ballPusher);
  ToggleSolenoid(/* rightJoystick.GetRawButtonPressed(2) */ false, hatchPusher);

}

void Robot::RunElevator() {
  double val = gamerJoystick.GetRawAxis(3); // right joy, up/down
  elevatorSparkMotor.Set(gamerJoystick.GetRawAxis(3)  * 0.2);
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


void Robot::GetDistances() {
  distances.lidarL = leftLidar->AquireDistance();
  distances.lidarR = rightLidar->AquireDistance();  

  // distances.ultrasonicL = leftUltrasonic->GetRangeInches();
  // distances.ultrasonicR = rightUltrasonic->GetRangeInches();
  distances.ultrasonicL = analogUltrasonicL->GetAverageValue();
  distances.ultrasonicR = analogUltrasonicR->GetAverageValue();
  // int gotValueR = analogUltrasonicR->GetValue();
  // int gotValueL = analogUltrasonicL->GetValue();
  

  // int bytesReceived = serialUltrasonic->GetBytesReceived();
  // char buffer[bytesReceived];
  // int readBytes = serialUltrasonic->Read(buffer, bytesReceived);

  // frc::SmartDashboard::PutNumber("bytesReceived", bytesReceived);
  // frc::SmartDashboard::PutRaw("buffer", buffer);
  // frc::SmartDashboard::PutNumber("readBytes", readBytes);

}
  

void Robot::Align(int left, int right, int tolerance, DistanceType type) {
  // double kP = 1.0;
  // double kI = 0.0;
  // double kD = 0.0;

  double dif = left - right;

  double forward  = -(+0.5);
  double backward = -(-0.5);

  // int newval = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;

  // (20, 120) and (200, 900)
  // left-right
  // 120-20 -- 20-120 = 100 - -100
  // 700 -- -700

  int go;
  if (type == lidar) {
    go = (dif - (120-20) ) / ((20-120) - (120-20) ) * (-0.85 - 0.85) + 0.85;
  } else if (type == ultrasonic) {
    go = (dif - (900-200) ) / ((200-900) - (900-200) ) * (-0.85 - 0.85) + 0.85;
  };

  if (left - right - tolerance > 0) {
    DriveLeft ( go);
    DriveRight(-go); 
  } else if (right - left - tolerance > 0) {
    DriveLeft (-go);
    DriveRight( go);
  } else {
    DriveBoth(0.0);
  }
}

void Robot::LidarInit() {}


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

  // frc::SmartDashboard::PutBoolean("line sensor left", lineSensorLeft->Get());
  // frc::SmartDashboard::PutBoolean("line sensor mid", lineSensorMid->Get());
  // frc::SmartDashboard::PutBoolean("line sensor right", lineSensorRight->Get());

  // frc::SmartDashboard::PutNumber("Left Encoder", leftEnc.GetDistance());
	// frc::SmartDashboard::PutNumber("Right Encoder", rightEnc.GetDistance());

  // frc::SmartDashboard::PutNumber("lineSensorLeft", lineSensorLeft->GetValue());
  // frc::SmartDashboard::PutNumber("lineSensorMid", lineSensorMid->GetValue());
  // frc::SmartDashboard::PutNumber("lineSensor2", lineSensor2->GetAverageValue());
  // frc::SmartDashboard::PutNumber("lineSensorRight", lineSensorRight->GetAverageValue());
  // frc::SmartDashboard::PutNumber("lineSensornavx4", lineSensornavx4->GetAverageValue());

  frc::SmartDashboard::PutNumber("dist.ultraR", distances.ultrasonicR);
  frc::SmartDashboard::PutNumber("dist.ultraL", distances.ultrasonicL);
  frc::SmartDashboard::PutNumber("dist.lidarR", distances.lidarR);
  frc::SmartDashboard::PutNumber("dist.lidarL", distances.lidarL);

  
}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif