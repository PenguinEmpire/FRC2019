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

  Spark_l1.SetInverted(true);
  Spark_l2.SetInverted(true);
  Spark_r1.SetInverted(false);
  Spark_r2.SetInverted(false);

  //-----------------------------------------------------------------------------------

/* some deprecated sensor stuff
  //Object Initialization
  // liftTop = new frc::DigitalInput(DIO_ELEVATOR_TOP);
  // liftMid = new frc::DigitalInput(DIO_ELEVATOR_MID);
  // liftBottom = new frc::DigitalInput(DIO_ELEVATOR_BOTTOM);
  // LineSensorInit();
  // serialUltrasonic->EnableTermination((char)31);
  // serialUltrasonic->Reset();
*/

  frc::SmartDashboard::PutNumber("HATCH_MID", elevatorHeights[HATCH_MID]);
  frc::SmartDashboard::PutNumber("HATCH_HIGH", elevatorHeights[HATCH_HIGH]);
  frc::SmartDashboard::PutNumber("BALL_LOW", elevatorHeights[BALL_LOW]);

  elevatorZero = new frc::DigitalInput(ELEVATOR_ZERO_HALL_DIO);

  analogUltrasonicR->InitAccumulator();
  analogUltrasonicR->ResetAccumulator();
  analogUltrasonicL->InitAccumulator();
  analogUltrasonicL->ResetAccumulator();
 

  leftEnc.SetDistancePerPulse(PULSE_IN);
  rightEnc.SetDistancePerPulse(PULSE_IN);

  leftEnc.Reset();
  rightEnc.Reset();

  compressor.SetClosedLoopControl(true);
  ShiftGears(Direction::down, driveGearboxes);
  intakeArm.Set(frc::DoubleSolenoid::kReverse);
  ballPusher.Set(frc::DoubleSolenoid::kReverse);    // IS THIS RIGHT? (TODO)
  hatchPusher.Set(frc::DoubleSolenoid::kReverse);

  TalonInit();
  intakeMotor.SetInverted(false);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { // runs after mode specific
  Testing();
  GetDistances();

  elevatorAtZero = !elevatorZero->Get();
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

  ahrs->Reset();
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
  RunElevator(); 
  intakeMotor.Set(gamerJoystick.GetRawAxis(1));

}

void Robot::TestPeriodic() {}

//--------------------------

void Robot::TalonInit() {
  l1.SetInverted(true);
  // l2.SetInverted(true);
  r1.SetInverted(false); 
  // r2.SetInverted(false);

  l2.Follow(l1);
  r2.Follow(r1);

  l1.SetNeutralMode(NeutralMode::Brake);
  r1.SetNeutralMode(NeutralMode::Brake);
  l2.SetNeutralMode(NeutralMode::Brake);
  r2.SetNeutralMode(NeutralMode::Brake);

  l1.ConfigOpenloopRamp(1.5, 10);
  r1.ConfigOpenloopRamp(1.5, 10);

  // r1.SetSafety

  elevator.SetNeutralMode(NeutralMode::Brake);
  elevator.SetInverted(true);
  elevator.SetSensorPhase(true); 

  elevator.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
}

void Robot::LineSensorInit() /* all commented out - deprecated */ {
  // lineSensorLeft = new frc::DigitalInput(7);  //  6);//10);
  // lineSensorMid = new frc::DigitalInput(8);  //  );//11);
  // lineSensorRight = new frc::DigitalInput(9);  //  (4);//12);
  
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

  toleranceScalar = ((leftJoystick.GetRawAxis(3) + 1) / 2);
  int ultraTol = 240 * toleranceScalar;
  int lidarTol =  80 * toleranceScalar;
  frc::SmartDashboard::PutNumber("ultraTol", ultraTol);
  frc::SmartDashboard::PutNumber("lidarTol", lidarTol);

  if (leftJoystick.GetRawButton(3)) { // align w/ ultrasonic
    Align(distances.ultrasonicL, distances.ultrasonicR, ultraTol, DistanceType::ultrasonic);
    frc::SmartDashboard::PutBoolean("appr-ultrasonic", true);
  } else if (leftJoystick.GetRawButton(4)) { // align w/ lidar
    Align(distances.lidarL, distances.lidarR, lidarTol, DistanceType::lidar);
    frc::SmartDashboard::PutBoolean("appr-lidar", true);
  } else if (leftJoystick.GetRawButton(5)) { // approach & align (?) w/ limelight
    GetLimelight();
    DriveLeft ( left_command * -0.7);
    DriveRight(right_command * -0.7);

  } else { // manual driving
    frc::SmartDashboard::PutBoolean("appr-ultrasonic", false);
    frc::SmartDashboard::PutBoolean("appr-lidar", false);

    // Joystick Inputs for driving
    if(fabs(left - right) < leftJoystick.GetRawAxis(3)) {
      both = (left + right) / 2;
      DriveBoth (calculateDampenedJoystick(both ));
      Spark_l1.Set(both);
      Spark_l2.Set(both);
      Spark_r1.Set(both);
      Spark_r2.Set(both);
    } else {
      DriveLeft (calculateDampenedJoystick(left ));
      Spark_l1.Set(left);
      Spark_l2.Set(left);
      DriveRight(calculateDampenedJoystick(right));
      Spark_r1.Set(right);
      Spark_r2.Set(right);
    }
  }

  ToggleSolenoid(rightJoystick.GetRawButtonPressed(2), driveGearboxes);
  ToggleSolenoid(rightJoystick.GetRawButtonPressed(3), ballPusher);
  ToggleSolenoid(rightJoystick.GetRawButtonPressed(5), intakeArm);
  ToggleSolenoid(rightJoystick.GetRawButtonPressed(4), hatchPusher);

  if (rightJoystick.GetRawButtonPressed(11)) {
    elevatorDestination = HATCH_MID;  
  } else if (rightJoystick.GetRawButtonPressed(12)) {
    elevatorDestination = MANUAL;
  } else if (rightJoystick.GetRawButtonPressed(9)) {
    elevatorDestination = HATCH_HIGH;
  }

  if (leftJoystick.GetRawButtonPressed(2)) {
    Spark_l1.SetInverted(!Spark_l1.GetInverted());
    Spark_l2.SetInverted(!Spark_l2.GetInverted());
    Spark_r1.SetInverted(!Spark_r1.GetInverted());
    Spark_r2.SetInverted(!Spark_r2.GetInverted());
  }
}

void Robot::RunElevator() {
  int curPos = elevator.GetSensorCollection().GetPulseWidthPosition();
  int absPos = elevator.GetSelectedSensorPosition() & 0xFFF;
  int absPos2 = elevator.GetSelectedSensorPosition();
  frc::SmartDashboard::PutNumber("curPos", curPos);
  frc::SmartDashboard::PutNumber("absPos", absPos);
  
  if (elevatorState == CALIBRATING) {
    printf("in elevator calibrating\n");
    if (!elevatorAtZero) {
      printf("in cali - not at zero\n");
      // elevator.Set(ControlMode::Velocity, -0.0001);
      elevator.Set(ControlMode::PercentOutput, -0.8);
    } else {
      printf("in cali - else (at zero)\n");
      elevator.Set(ControlMode::PercentOutput, 0.0);
      elevator.SetSelectedSensorPosition(absPos, 0, 10);
      elevator.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
      // elevator.SetSelectedSensorPosition(curPos, 0, 10);
      elevatorState = NORMAL;
    }
  } else if (elevatorState == NORMAL) { //MANUAL, MECHANICAL_LOW, PICKUP, BALL_CARGO, HATCH_CARGO, HATCH_LOW, HATCH_MID, HATCH_HIGH, BALL_LOW, BALL_MID, BALL_HIGH
    int _pos;
    printf("in elevator normal\n");
    switch (elevatorDestination) {
      case MECHANICAL_LOW:
      case PICKUP:
      case BALL_CARGO:
      case HATCH_CARGO:
      case HATCH_LOW:
        break;
      case HATCH_MID:
        _pos = dash->GetNumber("HATCH_MID", elevatorHeights[HATCH_MID]);
        elevator.Set(ControlMode::Position, _pos);
        break;
      case HATCH_HIGH:
        _pos = dash->GetNumber("HATCH_HIGH", elevatorHeights[HATCH_HIGH]);
        elevator.Set(ControlMode::Position, _pos);
        break;
      case BALL_LOW:
        _pos = dash->GetNumber("BALL_LOW", elevatorHeights[BALL_LOW]);
        elevator.Set(ControlMode::Position, _pos);
        break;
      case BALL_MID:
      case BALL_HIGH:
        printf("setting to a preset\n");
        // if (curPos < elevatorHeights[elevatorDestination] - 100) {
        //   elevator.Set(ControlMode::PercentOutput, 0.6);
        // } else if (curPos > elevatorHeights[elevatorDestination] + 100) {
        //   elevator.Set(ControlMode::PercentOutput, -0.6);
        // } else {
        //   elevator.Set(ControlMode::PercentOutput, 0.0);
        //   elevatorDestination = MANUAL;
        // }
        elevator.Set(ControlMode::Position, elevatorHeights[elevatorDestination]);
        // elevator.Set(ControlMode::Position, 200);
        break;
      case HOLD:
        elevator.Set(ControlMode::PercentOutput, 0.2);
        break;
      case MANUAL:
        printf("setting to gamerjoystick\n");
        elevator.Set(ControlMode::PercentOutput, -gamerJoystick.GetRawAxis(5) + 0.2);
        break;
      default:
        elevator.Set(ControlMode::PercentOutput, 0.0);
        break;
    }
    if (elevatorAtZero) {
      elevator.ConfigPeakOutputReverse(0);
    } else {
      elevator.ConfigPeakOutputReverse(-1);
    }
    // auto x = new int[2]{0, -1};
    // elevator.ConfigPeakOutputReverse(new int[2]{0, -1}[!elevatorAtZero])
  } 
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
  // LIDAR
  distances.lidarL = leftLidar->AquireDistance();
  distances.lidarR = rightLidar->AquireDistance();  

  // ULTRASONIC
  //   ultrasonic class
  // distances.ultrasonicL = leftUltrasonic->GetRangeInches();
  // distances.ultrasonicR = rightUltrasonic->GetRangeInches();
  //   analoginput class
  distances.ultrasonicL = analogUltrasonicL->GetAverageValue();
  distances.ultrasonicR = analogUltrasonicR->GetAverageValue();
  // int gotValueR = analogUltrasonicR->GetValue();
  // int gotValueL = analogUltrasonicL->GetValue();
  
/* serial ultrasonic class
  // int bytesReceived = serialUltrasonic->GetBytesReceived();
  // char buffer[bytesReceived];
  // int readBytes = serialUltrasonic->Read(buffer, bytesReceived);

  // frc::SmartDashboard::PutNumber("bytesReceived", bytesReceived);
  // frc::SmartDashboard::PutRaw("buffer", buffer);
  // frc::SmartDashboard::PutNumber("readBytes", readBytes);
*/
}

void Robot::GetLimelight() {
  double targetOffsetAngle_Horizontal = limelight->GetNumber("tx", 0.0);
  double targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
  double targetArea = limelight->GetNumber("ta", 0.0);
  double targetSkew = limelight->GetNumber("ts", 0.0);

  /** ESTIMATING DISTANCE
   * d = (h2-h1) / tan(a1+a2)
   * where
   *  h2 = height of target
   *  h1 = height of camera lens
   *  a1 = angle of the camera above the ground
   *  a2 = y angle to the target (get from limelight)
   * see http://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera
   */
  //

/* copied from them
  float KpAim = -0.1f;
  float KpDistance = -0.1f;
  float min_aim_command = 0.05f;

  float tx = limelight->GetNumber("tx", 0.0);
  float ty = limelight->GetNumber("ty", 0.0);

  float heading_error  = -tx;
  float distance_error = -ty;
  float steering_adjust = 0.0f;

  if (tx > 1.0) {
    steering_adjust = KpAim * heading_error - min_aim_command;
  } else if (tx < 1.0) {
    steering_adjust = KpAim * heading_error + min_aim_command;
  }

  float distance_adjust = KpDistance * distance_error;

  left_command  += steering_adjust + distance_adjust;
  right_command -= steering_adjust + distance_adjust;
  frc::SmartDashboard::PutNumber("left_command", left_command);
  frc::SmartDashboard::PutNumber("right_command", right_command);
*/

  float tx = limelight->GetNumber("tx", 0.0);
  float ty = limelight->GetNumber("ty", 0.0);

  double P_align = 0.15 * toleranceScalar;
  left_command  = -( (P_align)/* 0.1 */ * tx + (0.2 * (ty + 0.1)));
  right_command = -(-(P_align)/* 0.1 */ * tx + (0.2 * (ty + 0.1)));

  // double temp = left_command;
  // left_command = right_command;
  // right_command = temp;
  

  frc::SmartDashboard::PutNumber("left_command", left_command);
  frc::SmartDashboard::PutNumber("right_command", right_command);
  frc::SmartDashboard::PutNumber("P_align", P_align);

}  

void Robot::Align(int left, int right, int tolerance, Robot::DistanceType type) {
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

  double go;
  if (type == lidar) {
    go = linearMap(dif, -100, 100, 0.85, -0.85);
  } else if (type == ultrasonic) {
    go = linearMap(dif, -700, 700, 0.85, -0.85); //(dif - (900-200) ) / ((200-900) - (900-200) ) * (-0.85 - 0.85) + 0.85;
  };

  frc::SmartDashboard::PutNumber("dis| go", go);
  frc::SmartDashboard::PutNumber("dis| dif", dif);

  if (fabs(dif) > tolerance) {
    DriveLeft ( go);
    DriveRight(-go);
  }

  // if (left - right  > tolerance) {
  //   DriveLeft ( go);
  //   DriveRight(-go); 
  // } else if (right - left > tolerance) {
  //   DriveLeft (-go);
  //   DriveRight( go);
  // } else {
  //   DriveBoth(0.0);
  // }
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

double Robot::linearMap(double n, double start1, double stop1, double start2, double stop2) {
  /* double newval = */ return (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
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

  frc::SmartDashboard::PutBoolean("dio elevator", elevatorZero->Get());
  frc::SmartDashboard::PutBoolean("elevatorAtZero", elevatorAtZero);



  // frc::SmartDashboard::PutNumber("lineSensorLeft", lineSensorLeft->GetValue());
  // frc::SmartDashboard::PutNumber("lineSensorMid", lineSensorMid->GetValue());
  // frc::SmartDashboard::PutNumber("lineSensor2", lineSensor2->GetAverageValue());
  // frc::SmartDashboard::PutNumber("lineSensorRight", lineSensorRight->GetAverageValue());
  // frc::SmartDashboard::PutNumber("lineSensornavx4", lineSensornavx4->GetAverageValue());

  frc::SmartDashboard::PutNumber("dist.ultraR", distances.ultrasonicR);
  frc::SmartDashboard::PutNumber("dist.ultraL", distances.ultrasonicL);
  frc::SmartDashboard::PutNumber("dist.lidarR", distances.lidarR);
  frc::SmartDashboard::PutNumber("dist.lidarL", distances.lidarL);

  frc::SmartDashboard::PutString("elevatorState", elevatorState == CALIBRATING ? "cali" : (elevatorState == NORMAL ? "norm" : "other"));
  frc::SmartDashboard::PutString("elevatorDestination", elevatorDestination == MANUAL ? "manual" : (elevatorDestination == HOLD ? "hold" : (elevatorDestination == HATCH_MID ? "hatch_mid" : "other")));

  frc::SmartDashboard::PutNumber("gamer-5", gamerJoystick.GetRawAxis(5));
  
  // frc::SmartDashboard::PutNumber( "ahrs->GetPitch()                ", ahrs->GetPitch()                );
  // frc::SmartDashboard::PutNumber( "ahrs->GetRoll()                 ", ahrs->GetRoll()                 );
  // frc::SmartDashboard::PutNumber( "ahrs->GetYaw()                  ", ahrs->GetYaw()                  );
  // frc::SmartDashboard::PutBoolean("ahrs->IsCalibrating()           ", ahrs->IsCalibrating()           );
  // frc::SmartDashboard::PutNumber( "ahrs->GetWorldLinearAccelX()    ", ahrs->GetWorldLinearAccelX()    );
  // frc::SmartDashboard::PutNumber( "ahrs->GetWorldLinearAccelY()    ", ahrs->GetWorldLinearAccelY()    );
  // frc::SmartDashboard::PutNumber( "ahrs->GetWorldLinearAccelZ()    ", ahrs->GetWorldLinearAccelZ()    );
  // frc::SmartDashboard::PutBoolean("ahrs->IsMoving()                ", ahrs->IsMoving()                );
  // frc::SmartDashboard::PutBoolean("ahrs->IsRotating()              ", ahrs->IsRotating()              );
  // frc::SmartDashboard::PutBoolean("ahrs->IsMagnetometerCalibrated()", ahrs->IsMagnetometerCalibrated());
  // frc::SmartDashboard::PutNumber( "ahrs->GetAngle()                ", ahrs->GetAngle()                );
  // frc::SmartDashboard::PutNumber( "ahrs->GetRate()                 ", ahrs->GetRate()                 );
  // frc::SmartDashboard::PutNumber( "ahrs->GetVelocityX()            ", ahrs->GetVelocityX()            );
  // frc::SmartDashboard::PutNumber( "ahrs->GetVelocityY()            ", ahrs->GetVelocityY()            );
  // frc::SmartDashboard::PutNumber( "ahrs->GetVelocityZ()            ", ahrs->GetVelocityZ()            );
  
  
}



#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif