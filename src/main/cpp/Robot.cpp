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
  SD::PutData("Auto Modes", &m_chooser);

  //-----------------------------------------------------------------------------------

  /* some deprecated sensor stuff
    //Object Initialization
    // liftTop = new frc::DigitalInput(DIO_ELEVATOR_TOP);
    // liftMid = new frc::DigitalInput(DIO_ELEVATOR_MID);
    // liftBottom = new frc::DigitalInput(DIO_ELEVATOR_BOTTOM);
    // LineSensorInit();
    // #if ULTRA_EXIST
    // serialUltrasonic->EnableTermination((char)31);
    // serialUltrasonic->Reset();
    // #endif
  */

  SD::PutNumber("HATCH_MID", elevatorHeights[HATCH_MID]);
  SD::PutNumber("HATCH_HIGH", elevatorHeights[HATCH_HIGH]);
  SD::PutNumber("BALL_LOW", elevatorHeights[BALL_LOW]);

  printf("robot init test\n");

  SensorInit();
  timer->Reset();
  timer->Start();

  compressor.SetClosedLoopControl(true);

  #if (!PNEUMATIC_OBJECT)
    printf("\nsetting pneumatic default directions\n\n\nsetting pneumatic default directions\n\n\nset pneumatic default directions\n\n");
    SetPneumaticDefaultDirections();
  #endif

  TalonInit();
  #if COMP_ROBOT
    intakeMotor.SetInverted(false); // TODO_COMP
  #else
    intakeMotor.SetInverted(false);
  #endif
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
  #if PROFILING
  now = timer->Get();
  prev_now = now;
  start_now = now;
  printf("Starting RobotPeriodic : %f\n", now);
  #endif

  Testing();

  #if PROFILING
  now = timer->Get();
  // printf("Ran Testing : %f", now);
  printf("Running Testing took : %f\n", now - prev_now);
  prev_now = now;
  #endif

  GetDistances();

  #if PROFILING
  now = timer->Get();
  // printf("Got distances : %f", now);
  printf("Getting distances took : %f\n", now - prev_now);
  prev_now = now;
  #endif

  #if ELEVATOR_SENSOR_EXIST
    elevatorAtZero = !elevatorZero->Get();
  #else
    elevatorAtZero = (absPos2 < 3);
  #endif

  #if PROFILING
    now = timer->Get();
    // printf("Updated `elevatorAtZero` : %f", now);
    printf("Updating `elevatorAtZero` took : %f\n", now - prev_now);
    prev_now = now;
  #endif


  #if PNEUMATIC_OBJECT
    UpdatePneumatics();

    #if PROFILING
      now = timer->Get();
      // printf("Updated pneumatics objects : %f", now);
      printf("Updating pneumatics objects took : %f\n", now - prev_now);
      prev_now = now;
    #endif
  #endif

  masterLoopCount += 1;

  // printf("\n");

  #if PROFILING
  printf("TeleopPeriodic took : %f\n", now - start_now);
  #endif
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

  // leftEnc.Reset();  \
  // rightEnc.Reset(); | // TODO?
  //                   | Are these necessary?
  // ahrs->Reset();    /
}

void Robot::AutonomousPeriodic() {
  { // Built in auto code
    if (m_autoSelected == kAutoNameCustom) {
      // Custom Auto goes here
    } else {
      // Default Auto goes here
    }
  }

  TeleopPeriodic();
}

void Robot::TeleopInit() {}
 
void Robot::TeleopPeriodic() {
  #if PROFILING
  now = timer->Get();
  prev_now = now;
  start_now = now;
  printf("Starting TeleopPeriodic : %f\n", now);
  #endif

  HandleJoysticks();

  #if PROFILING
  now = timer->Get();
  // printf("Handled joysticks : %f", now);
  printf("Handling joysticks took : %f\n", now - prev_now);
  prev_now = now;
  #endif
  
  RunElevator();

  #if PROFILING
  now = timer->Get();
  // printf("Ran elevator : %f", now);
  printf("Running elevator took : %f\n", now - prev_now);
  prev_now = now;
  #endif

  intakeMotor.Set(gamerJoystick.GetRawAxis(1));

  #if PROFILING
  now = timer->Get();
  // printf("Set intake motor : %f", now);
  printf("Setting intake motor took : %f\n", now - prev_now);
  prev_now = now;
  #endif

  #if (LIDAR_EXIST || ULTRA_EXIST)
    StopForwardMovement();
  #endif

  #if PROFILING
  now = timer->Get();
  // printf("Stopped forward movement? : %f", now);
  printf("Stopping forward movement took : %f\n", now - prev_now);
  prev_now = now;

  printf("TeleopPeriodic took : %f\n\n", now - start_now);
  #endif
}

void Robot::TestPeriodic() {}

void Robot::DisabledPeriodic() {
  rightJoystick.GetRawButtonPressed(1);

  rightJoystick.GetRawButtonPressed(2);
  rightJoystick.GetRawButtonPressed(3);
  rightJoystick.GetRawButtonPressed(5);
  rightJoystick.GetRawButtonPressed(4);
  rightJoystick.GetRawButtonPressed(6);

  buttonJoystick.GetRawButtonPressed(1);
  buttonJoystick.GetRawButtonPressed(2);
  buttonJoystick.GetRawButtonPressed(3);
  buttonJoystick.GetRawButtonPressed(4);
  buttonJoystick.GetRawButtonPressed(5);
  buttonJoystick.GetRawButtonPressed(6);
  buttonJoystick.GetRawButtonPressed(7);
  buttonJoystick.GetRawButtonPressed(8);
  buttonJoystick.GetRawButtonPressed(8);
  buttonJoystick.GetRawButtonPressed(9);
  buttonJoystick.GetRawButtonPressed(10);
  buttonJoystick.GetRawButtonPressed(11);  
  buttonJoystick.GetRawButtonPressed(12);  

  gamerJoystick.GetRawButtonPressed(6);
  gamerJoystick.GetRawButtonPressed(1);
  gamerJoystick.GetRawButtonPressed(4);
  leftJoystick.GetRawButtonPressed(2);
}

//--------------------------

void Robot::TalonInit() {
  #if (COMP_ROBOT || PRACTICE_TALON)
    l1.ConfigFactoryDefault();
    l2.ConfigFactoryDefault();
    r1.ConfigFactoryDefault();
    r2.ConfigFactoryDefault();
  #endif

  elevator.ConfigFactoryDefault();

  elevator.ConfigOpenloopRamp(0.05);
  elevator.ConfigClosedloopRamp(0.02);

  elevator.Config_kP(0, 6.0, 10);
  elevator.Config_kF(0, 0.25, 10);
  elevator.Config_kD(0, 80, 10);


  #if (COMP_ROBOT || PRACTICE_TALON)
    l1.ConfigContinuousCurrentLimit(39, 10);
    // l2.ConfigContinuousCurrentLimit(39, 10);
    r1.ConfigContinuousCurrentLimit(39, 10);
    // r2.ConfigContinuousCurrentLimit(39, 10); 

    l1.ConfigPeakCurrentLimit(0, 10);
    // l2.ConfigPeakCurrentLimit(0, 10);
    r1.ConfigPeakCurrentLimit(0, 10);
    // r2.ConfigPeakCurrentLimit(0, 10);  

    r1.EnableCurrentLimit(true);
    r1.EnableCurrentLimit(true);  

    l1.SetInverted(false);
    l2.SetInverted(false);
    r1.SetInverted(true); 
    r2.SetInverted(true);

    l2.Follow(l1);
    r2.Follow(r1);

    l1.SetNeutralMode(NeutralMode::Brake);
    r1.SetNeutralMode(NeutralMode::Brake);
    l2.SetNeutralMode(NeutralMode::Brake);
    r2.SetNeutralMode(NeutralMode::Brake);

    #if OPEN_LOOP_RAMP
      l1.ConfigOpenloopRamp(DRIVE_OPENLOOP_RAMP, 10);
      r1.ConfigOpenloopRamp(DRIVE_OPENLOOP_RAMP, 10);
    #endif

    // r1.SetSafety

  #else
    Spark_l1.SetInverted(false);
    Spark_l2.SetInverted(false);
    Spark_r1.SetInverted(true);
    Spark_r2.SetInverted(true); 
  #endif

  elevator.ConfigContinuousCurrentLimit(39, 10);
  elevator.ConfigPeakCurrentLimit(0, 10);    

  elevator.SetNeutralMode(NeutralMode::Brake);
  #if COMP_ROBOT
    elevator.SetInverted(false);
    elevator.SetSensorPhase(false); 
  #else
    elevator.SetInverted(true); 
    elevator.SetSensorPhase(true); 
  #endif

  elevator.ConfigPeakOutputReverse(-1.0);
  elevator.ConfigPeakOutputForward(1.0);


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
  #if (COMP_ROBOT || PRACTICE_TALON)
    l1.Set(ControlMode::PercentOutput, amount);
  #else
    Spark_l1.Set(amount);
    Spark_l2.Set(amount);
  #endif
}
void Robot::DriveRight(double amount) {
  #if (COMP_ROBOT || PRACTICE_TALON)
    r1.Set(ControlMode::PercentOutput, amount);
  #else
    Spark_r1.Set(amount);
    Spark_r2.Set(amount);
  #endif
}
void Robot::TurnLeft(double amount) {
  #if (COMP_ROBOT || PRACTICE_TALON)
    l1.Set(ControlMode::PercentOutput, -amount);
    r1.Set(ControlMode::PercentOutput,  amount);
  #else
    Spark_l1.Set(-amount);
    Spark_l2.Set(-amount);
    Spark_r1.Set( amount);
    Spark_r2.Set( amount);
  #endif

}
void Robot::TurnRight(double amount) {
  #if (COMP_ROBOT || PRACTICE_TALON)
    l1.Set(ControlMode::PercentOutput,  amount);
    r1.Set(ControlMode::PercentOutput, -amount);
  #else
    Spark_l1.Set( amount);
    Spark_l2.Set( amount);
    Spark_r1.Set(-amount);
    Spark_r2.Set(-amount);
  #endif
}
void Robot::DriveBoth(double amount) {
  DriveLeft (amount);
  DriveRight(amount);
}
void Robot::HandleJoysticks() {
  double left  = -leftJoystick.GetRawAxis(1);
  double right = -rightJoystick.GetRawAxis(1);
  SD::PutNumber("left", left);
  SD::PutNumber("right", right);
  double both;

  if (driveInverted) {
    double temp = left;
    left = right;
    right = temp;
  }

  toleranceScalar = ((leftJoystick.GetRawAxis(3) + 1) / 2);
  int ultraTol = 240 * toleranceScalar;
  int lidarTol =  80 * toleranceScalar;
  SD::PutNumber("ultraTol", ultraTol);
  SD::PutNumber("lidarTol", lidarTol);

  #if LIMELIGHT_EXIST
    GetLimelight();
  #endif

  #if (OPEN_LOOP_RAMP && (COMP_ROBOT || PRACTICE_TALON))
    l1.ConfigOpenloopRamp(0.0);
    r1.ConfigOpenloopRamp(0.0);
  #endif

  if (leftJoystick.GetRawButton(3)) { // align w/ ultrasonic
    #if ULTRA_EXIST
      Align(DistanceType::ULTRASONIC);
      SD::PutBoolean("appr-ultrasonic", true);
    #else
      printf("ultra align not possible: line %i\n", __LINE__);
    #endif
  } else if (leftJoystick.GetRawButton(4)) { // align w/ lidar
    #if LIDAR_EXIST
      Align(DistanceType::LIDAR);
      SD::PutBoolean("appr-lidar", true);
    #else
      printf("lidar align not possible: line %i\n", __LINE__);
    #endif
  } else if (leftJoystick.GetRawButton(5)) { // approach & align (?) w/ limelight
    #if LIMELIGHT_EXIST
      DriveLeft ( left_command * 0.7);
      DriveRight(right_command * 0.7);
    #else
      printf("limelight align/approach not allowed: line %i\n", __LINE__);
    #endif
  } else if (leftJoystick.GetRawButton(6)) {
    printf("calling align(navx)\n");
    Align(DistanceType::NAVX);
  } else { // manual driving
    #if ((COMP_ROBOT || PRACTICE_TALON) && OPEN_LOOP_RAMP)
      l1.ConfigOpenloopRamp(DRIVE_OPENLOOP_RAMP, 10);
      r1.ConfigOpenloopRamp(DRIVE_OPENLOOP_RAMP, 10);
    #endif

    SD::PutBoolean("appr-ultrasonic", false);
    SD::PutBoolean("appr-lidar", false);

    // Joystick Inputs for driving
    if(fabs(left - right) < 0.1 /* rightJoystick.GetRawAxis(3) */ ) { // !!!!!! - TODO?
      both = (left + right) / 2;
      DriveBoth (calculateDampenedJoystick(both ));
    } else {
      DriveLeft (calculateDampenedJoystick(left ));
      DriveRight(calculateDampenedJoystick(right));
    }
  }

  #if PNEUMATIC_OBJECT 
    driveGearboxes.ToggleOverride(rightJoystick.GetRawButtonPressed(2));;
    intakeArm.ToggleOverride(rightJoystick.GetRawButtonPressed(3));
    ballPusher.ToggleOverride(rightJoystick.GetRawButtonPressed(6));
    // ballPusher.ToggleTimed(2.5, rightJoystick.GetRawButtonPressed(5));
    #if COMP_ROBOT
      hatchPusher.ToggleTimed(2.5, rightJoystick.GetRawButtonPressed(4));
      jumper.Toggle(rightJoystick.GetRawButtonPressed(6));
      ballPusher.ToggleTimed(2.5, rightJoystick.GetRawButtonPressed(5));
    #else
      ballPusher.ToggleTimed(2.5, rightJoystick.GetRawButtonPressed(4));
    #endif
  #else // TODO??
    ToggleSolenoid(rightJoystick.GetRawButtonPressed(2), driveGearboxes);
    ToggleSolenoid(rightJoystick.GetRawButtonPressed(4), ballPusher);
    ToggleSolenoid(rightJoystick.GetRawButtonPressed(5), intakeArm);
    #if COMP_ROBOT
      ToggleSolenoid(rightJoystick.GetRawButtonPressed(3), hatchPusher);
      ToggleSolenoid(rightJoystick.GetRawButtonPressed(6), jumper);
    #endif
  #endif

  if (rightJoystick.GetRawButtonPressed(1)) {
    ahrs->Reset();
  }

  ChooseElevatorMode();
  ChooseAlignMode();

  if (leftJoystick.GetRawButtonPressed(2)) {
    #if (COMP_ROBOT || PRACTICE_TALON)
      l1.SetInverted(!l1.GetInverted());
      l2.SetInverted(!l2.GetInverted());
      r1.SetInverted(!r1.GetInverted());
      r2.SetInverted(!r2.GetInverted());
    #else
      Spark_l1.SetInverted(!Spark_l1.GetInverted());
      Spark_l2.SetInverted(!Spark_l2.GetInverted()); 
      Spark_r1.SetInverted(!Spark_r1.GetInverted());
      Spark_r2.SetInverted(!Spark_r2.GetInverted());
    #endif

    driveInverted = !driveInverted;
  }
}

void Robot::RunElevator() {
  #if DO_EXTRA_IO
    curPos = elevator.GetSensorCollection().GetPulseWidthPosition();
    absPos = elevator.GetSelectedSensorPosition() & 0xFFF;
    SD::PutNumber("curPos", curPos);
    SD::PutNumber("absPos", absPos);
  #endif
  absPos2 = elevator.GetSelectedSensorPosition();
  SD::PutNumber("absPos2", absPos2);

  SD::PutString("elevatorState", elevatorStateNames[elevatorState]);
  SD::PutString("elevatorDestination", elevatorDestinationNames[elevatorDestination]);
  SD::PutNumber("current elevator target", elevatorHeights[elevatorDestination]);
  if (elevator.GetControlMode() == ctre::phoenix::motorcontrol::ControlMode::Position) {
    SD::PutNumber("talon's target", elevator.GetClosedLoopTarget());
  } else {
    SD::PutString("talon's target", "N/A");
  }


  if (elevatorState == CALIBRATING) {
    CalibrateElevator();
  } else if (elevatorState == NORMAL) { //MANUAL, MECHANICAL_LOW, PICKUP, BALL_CARGO, HATCH_CARGO, HATCH_LOW, HATCH_MID, HATCH_HIGH, BALL_LOW, BALL_MID, BALL_HIGH
    int _pos;
    // printf("in elevator normal\n");
    switch (elevatorDestination) {
      case HOLD:
        elevator.Set(ControlMode::PercentOutput, ELEVATOR_FEEDFORWARD);
        break;
      case MANUAL:
        // printf("setting to gamerjoystick\n");
        elevator.Set(ControlMode::PercentOutput, (-gamerJoystick.GetRawAxis(5) /*+ ELEVATOR_FEEDFORWARD*/) );
        break;
      #if ELEVATOR_SENSOR_EXIST
        case DOWN:  
          elevator.Set(ControlMode::PercentOutput, ELEVATOR_DOWNSPEED);
          if (elevatorAtZero) {
            elevatorDestination = MANUAL;
          }
          break;
      #endif
      case MECHANICAL_LOW:
        break;
      case PICKUP:
        break;
      case HATCH_CARGO:
        break;
      case BALL_CARGO:
      case HATCH_MID:
        // _pos = dash->GetNumber("HATCH_MID", elevatorHeights[HATCH_MID]);
        // elevator.Set(ControlMode::Position, _pos);
        // break;
      case HATCH_HIGH:
        // _pos = dash->GetNumber("HATCH_HIGH", elevatorHeights[HATCH_HIGH]);
        // elevator.Set(ControlMode::Position, _pos);
        // break;
      case BALL_LOW:
        // _pos = dash->GetNumber("BALL_LOW", elevatorHeights[BALL_LOW]);
        // elevator.Set(ControlMode::Position, _pos);
        // break;
      case HATCH_LOW:
      case BALL_MID:
      case BALL_HIGH:
        if (masterLoopCount % 50 == 0) {
          printf("setting to a preset\n");
        }
      /* brute-force up or down at 0.6
        if (curPos < elevatorHeights[elevatorDestination] - 100) {
          elevator.Set(ControlMode::PercentOutput, 0.6);
        } else if (curPos > elevatorHeights[elevatorDestination] + 100) {
          elevator.Set(ControlMode::PercentOutput, -0.6);
        } else {
          elevator.Set(ControlMode::PercentOutput, 0.0);
          elevatorDestination = MANUAL;
        }
        */
        if (elevatorDestination == HATCH_LOW && elevatorAtZero) {
          elevator.Set(ControlMode::PercentOutput, 0.0);
        } else {
          elevator.Set(ControlMode::Position, elevatorHeights[elevatorDestination]);
        }
        break;
      default:
        elevator.Set(ControlMode::PercentOutput, 0.0);
        break;
    }

    #if ELEVATOR_DOWN_PROTECTION
      if (elevatorAtZero) {
        elevator.ConfigPeakOutputReverse(0.0);
      } else {
        elevator.ConfigPeakOutputReverse(-1.0);
      }
      // auto x = new int[2]{0, -1};
      // elevator.ConfigPeakOutputReverse(new int[2]{0, -1}[!elevatorAtZero])
    #endif

    #if DO_PRINTF
      // if (masterLoopCount % 15 == 0) {
        // double percentOutput = elevator.GetMotorOutputPercent();
        // double voltageOutput = elevator.GetMotorOutputVoltage();
        // if (percentOutput > 2 || voltageOutput != 0.0) {
        //   // printf("elevator output: %f percent, %f volts", percentOutput, voltageOutput);
        // }
      // }
    #endif
  } 
}

void Robot::RunElevator2() {
  elevator.Set(ControlMode::PercentOutput, -gamerJoystick.GetRawAxis(5));
}

void Robot::ChooseElevatorMode() {
  ElevatorDestination prevDest = elevatorDestination;

  if (buttonJoystick.GetRawButtonPressed(7)) {
    elevatorDestination = HATCH_HIGH;  
  } else if (buttonJoystick.GetRawButtonPressed(8)) {
    elevatorDestination = BALL_HIGH;
  } else if (buttonJoystick.GetRawButtonPressed(9)) {
    elevatorDestination = HATCH_MID;
  } else if (buttonJoystick.GetRawButtonPressed(10)) {
    elevatorDestination = BALL_MID;
  } else if (buttonJoystick.GetRawButtonPressed(11) || gamerJoystick.GetRawButtonPressed(1)) {
    #if ELEVATOR_SENSOR_EXIST
      elevatorDestination = DOWN;
    #else
      elevatorDestination = HATCH_LOW;
    #endif
  } else if (buttonJoystick.GetRawButtonPressed(12)) {
    elevatorDestination = BALL_LOW;
  } else if (gamerJoystick.GetRawButtonPressed(6)) {
    elevatorDestination = MANUAL;
  } else if (gamerJoystick.GetRawButtonPressed(4)) {
    elevatorDestination = BALL_CARGO;
  } 

/* different PID profiles for going up and down
  if (prevDest == MANUAL || prevDest == HOLD) {
    elevator.SelectProfileSlot(1, 0); // less power - fill vals
  } else {
    if (elevatorDestination > prevDest) {
      elevator.SelectProfileSlot(0, 0);
    } else {
      elevator.SelectProfileSlot(1, 0);
    } 
  }
*/
}

void Robot::ChooseAlignMode() {
  if (buttonJoystick.GetRawButtonPressed(1)) {
    alignDestination = CARGO_FACE;
  } else if (buttonJoystick.GetRawButtonPressed(5)) {
    alignDestination = ROCKET_MID;
  } else if (buttonJoystick.GetRawButtonPressed(3)) {
    alignDestination = ROCKET_CLOSE;
  } else if (buttonJoystick.GetRawButtonPressed(6)) {
    alignDestination = ROCKET_FAR;
  } else if (buttonJoystick.GetRawButtonPressed(4)) {
    alignDestination = CARGO_SIDE;
  } else if (buttonJoystick.GetRawButtonPressed(2)) {
    alignDestination = HATCH_PICKUP;
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
}

void Robot::ToggleSolenoid(frc::DoubleSolenoid& solenoid) {
  frc::DoubleSolenoid::Value state = solenoid.Get();
  state = reverseStates[state];
  ShiftGears(state, solenoid);
}

void Robot::StopForwardMovement() {
  #if (WALL_PROTECTION && (LIDAR_EXIST || ULTRA_EXIST))
    Robot::distances distSource;
    int minDist;

    #if LIDAR_EXIST
      distSource = lidarDist;
      minDist = 8;
    #elif ULTRA_EXIST
      distSource = ultraDist;
      minDist = 250;
    #endif

    if (distSource.left < minDist || distSource.right < minDist) {
      l1.ConfigPeakOutputForward(0.0);
      r1.ConfigPeakOutputForward(0.0);
    } else {
      l1.ConfigPeakOutputForward(1.0);
      r1.ConfigPeakOutputForward(1.0);
    }
  #endif
}


void Robot::GetDistances() {
  // LIDAR
  #if LIDAR_EXIST
    lidarDist.left = leftLidar->AquireDistance();
    lidarDist.right = rightLidar->AquireDistance();  
  #endif

  // ULTRASONIC
  //   ultrasonic class
  // ultraDist.left  = leftUltrasonic->GetRangeInches();
  // ultraDist.right = rightUltrasonic->GetRangeInches();
  //   analoginput class
  #if ULTRA_EXIST
    ultraDist.left  = analogUltrasonicL->GetAverageValue();
    ultraDist.right = analogUltrasonicR->GetAverageValue();
  #endif
  // int gotValueR = analogUltrasonicR->GetValue();
  // int gotValueL = analogUltrasonicL->GetValue();
  
/* serial ultrasonic class
  // int bytesReceived = serialUltrasonic->GetBytesReceived();
  // char buffer[bytesReceived];
  // int readBytes = serialUltrasonic->Read(buffer, bytesReceived);

  // SD::PutNumber("bytesReceived", bytesReceived);
  // SD::PutRaw("buffer", buffer);
  // SD::PutNumber("readBytes", readBytes);
*/
}

void Robot::GetLimelight() {
  #if LIMELIGHT_EXIST
  // double targetOffsetAngle_Horizontal = limelight->GetNumber("tx", 0.0);
  // double targetOffsetAngle_Vertical = limelight->GetNumber("ty", 0.0);
  // double targetArea = limelight->GetNumber("ta", 0.0);
  // double targetSkew = limelight->GetNumber("ts", 0.0);

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
    SD::PutNumber("left_command", left_command);
    SD::PutNumber("right_command", right_command);
  */

  float tx = limelight->GetNumber("tx", 0.0);
  float ty = limelight->GetNumber("ty", 0.0);

  ty -= 2.5; // TODO: stronger?

  // P_align = 0.350 works well // TODO

  #define use_tolerance_scalar false

  #if use_tolerance_scalar
    double P_align = 0.35 * toleranceScalar;
    printf("using tolerance scalar\n");
  #else
    double P_align = 0.35;
  #endif
  
  left_command  = -(-(P_align)/* 0.1 */ * tx + (0.2 * (ty + 0.1)));
  right_command = -( (P_align)/* 0.1 */ * tx + (0.2 * (ty + 0.1)));

  left_command  /= 6.0;
  right_command /= 6.0; 

  SD::PutNumber("left_command", left_command);
  SD::PutNumber("right_command", right_command);
  SD::PutNumber("P_align", P_align);
  #endif
}  

/* deprecated */
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
  if (type == LIDAR) {
    go = linearMap(dif, -100, 100, 0.85, -0.85);
  } else if (type == ULTRASONIC) {
    go = linearMap(dif, -700, 700, 0.85, -0.85); //(dif - (900-200) ) / ((200-900) - (900-200) ) * (-0.85 - 0.85) + 0.85;
  };

  SD::PutNumber("dis| go", go);
  SD::PutNumber("dis| dif", dif);

  if (fabs(dif) > tolerance) {
    DriveLeft ( go);
    DriveRight(-go);
  }
}

void Robot::Align(Robot::DistanceType type) {
  if (masterLoopCount % 20 == 0) {
    printf("in align : type %s\n", distanceTypeNames[type].c_str());
  }
  // double kP = 1.0;
  // double kI = 0.0;
  // double kD = 0.0;

  int left;
  int right;
  int tolerance;

  if (type == LIDAR) {
    #if LIDAR_EXIST
      left  = lidarDist.left;
      right = lidarDist.right;
      tolerance = 80 * toleranceScalar;
    #else
      if (masterLoopCount % 20 == 0) {
        printf("no lidar - can't lidar align (line %i)\n", __LINE__);
      }
    #endif
  } else if (type == ULTRASONIC) {
    #if ULTRA_EXIST
      left  = ultraDist.left;
      right = ultraDist.right;
      tolerance = 240 * toleranceScalar;
    #else
      if (masterLoopCount % 20 == 0) {
        printf("no ultra - can't ultra align (line %i)\n", __LINE__);
      }
    #endif
  }

  double dif = left - right;

  double forward  = -(+0.5);
  double backward = -(-0.5);

  // int newval = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;

  // (20, 120) and (200, 900)
  // left-right
  // 120-20 -- 20-120 = 100 - -100
  // 700 -- -700

  double go;
  if (type == LIDAR) {
    #if LIDAR_EXIST
      printf("in lidar align\n");
      go = linearMap(dif, -100, 100, 0.85, -0.85);
    #endif
  } else if (type == ULTRASONIC) {
    #if ULTRA_EXIST
      printf("in ultra align\n");
      go = linearMap(dif, -700, 700, 0.85, -0.85); //(dif - (900-200) ) / ((200-900) - (900-200) ) * (-0.85 - 0.85) + 0.85;
    #endif
  } else if (type == NAVX) {
    printf("in navx align\n");

    double deg = ahrs->GetYaw();
    double speed = 0.65;
    double alignTol = 4.25;

    int angleScalar;
    if (gamerJoystick.GetRawButton(1)) {
      angleScalar = -1;
    } else {
      angleScalar = 1;
    }

    double target = angleScalar * alignAngles[alignDestination];

    printf("target: %f", target);

    if (-180 < deg && deg < target - alignTol) {
      printf("turning right to %f\n", target);
      TurnRight(speed);
    } else if (target + alignTol < deg) {
      printf("turning left to %f\n", target);
      TurnLeft(speed);
    // } 
    // else if (0 < deg && deg < 85) {
    //   printf("turning right to +90\n");
    //   TurnRight(speed);
    // } else if (95 < deg && deg < 180) {
    //   printf("turning left to +90\n");
    //   TurnLeft(speed);
    } else {
      DriveBoth(0.0);
      printf("what?? (aligned?)\n");
    }
  }


  SD::PutNumber("dis| go", go);
  SD::PutNumber("dis| dif", dif);

  if ((type == LIDAR && LIDAR_EXIST) || (type == ULTRASONIC && ULTRA_EXIST)) {
    printf("actual align\n");
    if (fabs(dif) > tolerance) {
      DriveLeft (-go);
      DriveRight( go);
    }
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

double Robot::linearMap(double n, double start1, double stop1, double start2, double stop2) {
  /* double newval = */ return (n - start1) / (stop1 - start1) * (stop2 - start2) + start2;
}

void Robot::Testing() {
  // bool gotTopButton = leftJoystick.GetTop(); 
  // bool gotTriggerButton = leftJoystick.GetTrigger();
  // SD::PutBoolean("topButtonLeft", gotTopButton);
  // SD::PutBoolean("TriggerButtonLeft", gotTriggerButton);
  
  // SD::PutBoolean("should be shifting", \
                                  leftJoystick.GetRawButton(6) || leftJoystick.GetRawButton(4));

  // SD::PutBoolean("line sensor left", lineSensorLeft->Get());
  // SD::PutBoolean("line sensor mid", lineSensorMid->Get());
  // SD::PutBoolean("line sensor right", lineSensorRight->Get());

  // SD::PutNumber("Left Encoder", leftEnc.GetDistance());
	// SD::PutNumber("Right Encoder", rightEnc.GetDistance());

  #if ELEVATOR_SENSOR_EXIST
    SD::PutBoolean("dio elevator", elevatorZero->Get());
  #endif
    SD::PutBoolean("elevatorAtZero", elevatorAtZero);
    SD::PutBoolean("elevator sensor exist", ELEVATOR_SENSOR_EXIST);

  // SD::PutNumber("lineSensorLeft", lineSensorLeft->GetValue());
  // SD::PutNumber("lineSensorMid", lineSensorMid->GetValue());
  // SD::PutNumber("lineSensor2", lineSensor2->GetAverageValue());
  // SD::PutNumber("lineSensorRight", lineSensorRight->GetAverageValue());
  // SD::PutNumber("lineSensornavx4", lineSensornavx4->GetAverageValue());

  #if ULTRA_EXIST
    SD::PutNumber("dist.ultraR", ultraDist.left);
    SD::PutNumber("dist.ultraL", ultraDist.right);
  #endif

  #if LIDAR_EXIST
    SD::PutNumber("dist.lidarR", lidarDist.left);
    SD::PutNumber("dist.lidarL", lidarDist.right);
  #endif

  SD::PutNumber("gamer-5", gamerJoystick.GetRawAxis(5));
  
  // SD::PutNumber( "ahrs->GetPitch()                ", ahrs->GetPitch()                );
  // SD::PutNumber( "ahrs->GetRoll()                 ", ahrs->GetRoll()                 );
  SD::PutNumber( "ahrs->GetYaw()                  ", ahrs->GetYaw()                  );
  // SD::PutBoolean("ahrs->IsCalibrating()           ", ahrs->IsCalibrating()           );
  #if DO_EXTRA_IO
  SD::PutNumber( "ahrs->GetWorldLinearAccelX()    ", ahrs->GetWorldLinearAccelX()    );
  SD::PutNumber( "ahrs->GetWorldLinearAccelY()    ", ahrs->GetWorldLinearAccelY()    );
  SD::PutNumber( "ahrs->GetWorldLinearAccelZ()    ", ahrs->GetWorldLinearAccelZ()    );
  SD::PutBoolean("ahrs->IsMoving()                ", ahrs->IsMoving()                );
  SD::PutBoolean("ahrs->IsRotating()              ", ahrs->IsRotating()              );
  // SD::PutBoolean("ahrs->IsMagnetometerCalibrated()", ahrs->IsMagnetometerCalibrated());
  SD::PutNumber( "ahrs->GetAngle()                ", ahrs->GetAngle()                );
  SD::PutNumber( "ahrs->GetRate()                 ", ahrs->GetRate()                 );
  SD::PutNumber( "ahrs->GetVelocityX()            ", ahrs->GetVelocityX()            );
  SD::PutNumber( "ahrs->GetVelocityY()            ", ahrs->GetVelocityY()            );
  SD::PutNumber( "ahrs->GetVelocityZ()            ", ahrs->GetVelocityZ()            );
  #endif
}

void Robot::SetPneumaticDefaultDirections() {
  #if (!PNEUMATIC_OBJECT)
    #if COMP_ROBOT
      ShiftGears(Direction::down, driveGearboxes); // TODO_COMP: remove?
      driveGearboxes.Set(frc::DoubleSolenoid::kReverse);
      intakeArm.Set(frc::DoubleSolenoid::kReverse);
      ballPusher.Set(frc::DoubleSolenoid::kReverse);
      hatchPusher.Set(frc::DoubleSolenoid::kForward);
      jumper.Set(frc::DoubleSolenoid::kForward);
    #else // TODO
      // ShiftGears(Direction::down, driveGearboxes);
      driveGearboxes.Set(frc::DoubleSolenoid::kReverse);
      intakeArm.Set(frc::DoubleSolenoid::kReverse);
      ballPusher.Set(frc::DoubleSolenoid:: kForward);  // good?
      // hatchPusher.Set(frc::DoubleSolenoid::kForward); // good?
      // jumper.Set(frc::DoubleSolenoid::kForward);
    #endif
  #endif
}

void Robot::SensorInit() {
  #if ELEVATOR_SENSOR_EXIST
    elevatorZero = new frc::DigitalInput(ELEVATOR_ZERO_HALL_DIO);
  #endif

  #if ULTRA_EXIST
    analogUltrasonicR->InitAccumulator();
    analogUltrasonicR->ResetAccumulator();
    analogUltrasonicL->InitAccumulator();
    analogUltrasonicL->ResetAccumulator();
  #endif

  leftEnc.SetDistancePerPulse(PULSE_IN);
  rightEnc.SetDistancePerPulse(PULSE_IN);

  leftEnc.Reset();
  rightEnc.Reset();
}

void Robot::UpdatePneumatics() {
  #if PNEUMATIC_OBJECT
    ballPusher.PenguinUpdate();
    #if COMP_ROBOT
      hatchPusher.PenguinUpdate();
      // jumper.PenguinUpdate();
    #endif

    // if (ballPusher.busy) {
    //   printf("ballPusher busy\n");
    // } else {printf("ballPusher not busy\n");}
  #endif
}

void Robot::CalibrateElevator() {
  #if ELEVATOR_SENSOR_EXIST
    elevatorCalibratingLoopCount += 1;
    // printf("in elevator calibrating\n"); 

    if (elevatorCalibratingLoopCount < 400) {
      if (!elevatorAtZero) {
        if (masterLoopCount & 25 == 0) {
          printf("in cali - not at zero\n");
        }
        // elevator.Set(ControlMode::Velocity, -0.0001);
        elevator.Set(ControlMode::PercentOutput, ELEVATOR_DOWNSPEED);
      } else {
        printf("in cali - at zero\n");
        SetElevatorAtZero();
      }
    } else {
      printf("in cali too long - restart robot or robot code to gain control of elevator\n");
      elevator.Set(ControlMode::PercentOutput, 0.0);
    }
  #else
    printf("warning - assuming elevator at zero\n");
    SetElevatorAtZero();
  #endif
}

void Robot::SetElevatorAtZero() {
  elevator.Set(ControlMode::PercentOutput, 0.0);
  // elevator.SetSelectedSensorPosition(0, 0, 10);
  elevator.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
  elevator.SetSelectedSensorPosition(0, 0, 10);
  elevatorState = NORMAL;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif