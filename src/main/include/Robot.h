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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "PenguinConstants.h"
#include "PenguinJoystick.h"
#include "Lidar.h"

#if PNEUMATIC_OBJECT
  #include "Pneumatic.h"
#endif

#include "ctre/Phoenix.h"
#include "AHRS.h"
// #include "pathfinder.h"

using std::unordered_map;
typedef frc::DigitalInput DIO;
using SD = frc::SmartDashboard;

class Robot : public frc::TimedRobot {
 public:

  enum Direction {
    up, down, left, right, backward, forward
  } currentGear = down;

  #if !PNEUMATIC_OBJECT
  /*const static*/ unordered_map<frc::DoubleSolenoid::Value, frc::DoubleSolenoid::Value> reverseStates = {
    {frc::DoubleSolenoid::kReverse, frc::DoubleSolenoid::kForward}, 
    {frc::DoubleSolenoid::kForward, frc::DoubleSolenoid::kReverse}, 
  };
  #endif

  enum State {
    UNINITIALIZED,
    LINING_UP
  } currentState = UNINITIALIZED;

	unordered_map<Robot::State, std::string> stateNames = {
		{UNINITIALIZED, "UNINITIALIZED"},
		{LINING_UP, "LINING_UP,"}
	};

  enum ElevatorState {
    CALIBRATING,
    NORMAL
    // HOLD
  } elevatorState = CALIBRATING;

	unordered_map<Robot::ElevatorState, std::string> elevatorStateNames = {
		{CALIBRATING, "CALIBRATING"},
		{NORMAL, "NORMAL"}
	};

  enum ElevatorDestination {
    MECHANICAL_LOW,
    PICKUP,
    HATCH_CARGO,
    HATCH_LOW,
    BALL_LOW,
    BALL_CARGO,
    HATCH_MID,
    BALL_MID,
    HATCH_HIGH,
    BALL_HIGH,
    MANUAL,
    #if ELEVATOR_SENSOR_EXIST
    DOWN,
    #endif
    HOLD
    // MANUAL, MECHANICAL_LOW, PICKUP, BALL_CARGO, HATCH_CARGO, HATCH_LOW, HATCH_MID, HATCH_HIGH, BALL_LOW, BALL_MID, BALL_HIGH, HOLD
  } elevatorDestination = MANUAL;

	unordered_map<Robot::ElevatorDestination, std::string> elevatorDestinationNames = {
		{MECHANICAL_LOW, "MECHANICAL_LOW"},
		{PICKUP, "PICKUP"},
		{HATCH_CARGO, "HATCH_CARGO"},
		{HATCH_LOW, "HATCH_LOW"},
		{BALL_LOW, "BALL_LOW"},
		{BALL_CARGO, "BALL_CARGO"},
		{HATCH_MID, "HATCH_MID"},
		{BALL_MID, "BALL_MID"},
		{HATCH_HIGH, "HATCH_HIGH"},
		{BALL_HIGH, "BALL_HIGH"},
		{MANUAL, "MANUAL"},
    #if ELEVATOR_SENSOR_EXIST
		{DOWN, "DOWN"},
    #endif
    {HOLD, "HOLD"}
	};

  /** Generate these with this Python:
   * def enum_as_strings(enums, enum_class_name, newline_seperated):
        s = enums.strip()
        s = s.split(',' + ['', '\n'][newline_seperated])
        # s = s.strip(',')
        l1 = map(lambda _: _.strip(), s)
        l2 = []
        for state in l1:
          l2.append('\t\t{%s, "%s"},' % (state, state))
        s = '\n'.join(l2)
        s = s[:-1] + '\n'
        decl = "\tunordered_map<Robot::%s, std::string> %s = {\n" % (enum_class_name, enum_class_name[0].lower() + enum_class_name[1:] + "Names")
        r = decl + s + "\t};\n"
        print(r)
  */ 

  enum LimelightApproachState {
    APPROACHING, CLOSE, AT, TOO_FAR
  };

	unordered_map<Robot::LimelightApproachState, std::string> limelightApproachStateNames = {
		{APPROACHING, "APPROACHING"},
		{CLOSE, "CLOSE"},
		{AT, "AT"},
		{TOO_FAR, "TOO_FAR"}
	};

  enum AlignPosition {
    // CARGO_FACE, CARGO_LEFT, CARGO_RIGHT, ROCKET_CLOSE_LEFT, ROCKET_CLOSE_RIGHT, ROCKET_FAR_LEFT, ROCKET_FAR_RIGHT, 
    CARGO_FACE, HATCH_PICKUP, ROCKET_MID, CARGO_SIDE, ROCKET_CLOSE, ROCKET_FAR
  } alignDestination = CARGO_FACE;

	unordered_map<Robot::AlignPosition, std::string> alignPositionNames = {
		{CARGO_FACE, "CARGO_FACE"},
		{HATCH_PICKUP, "HATCH_PICKUP"},
		{ROCKET_MID, "ROCKET_MID"},
		{CARGO_SIDE, "CARGO_SIDE"},
		{ROCKET_CLOSE, "ROCKET_CLOSE"},
		{ROCKET_FAR, "ROCKET_FAR"}
	};

  unordered_map<Robot::ElevatorDestination, int> elevatorHeights = {
      // {MANUAL,          1 /* placeholder!!!! TODO */ },
      // {MECHANICAL_LOW,  3660 /* placeholder!!!! TODO */ },
      // {PICKUP,          1 /* placeholder!!!! TODO */ },
      // {HATCH_CARGO,     1 /* placeholder!!!! TODO */ },
      // {HOLD,            1 /* placeholder!!!! TODO */},

      {HATCH_LOW,  -1000}, // TODO : surely 0?
      {BALL_CARGO,      1 /* placeholder!!!! TODO */ },
      #if COMP_ROBOT
        {HATCH_MID,   9761}, //9761
        {HATCH_HIGH,  20000}, //20000
      #else
        {HATCH_MID,   12000},
        {HATCH_HIGH,  24200},
      #endif
      {BALL_LOW,    4100 },
      {BALL_MID,    16800},
      {BALL_HIGH,   28550}
  };

  unordered_map<Robot::AlignPosition, double> alignAngles {
    {CARGO_FACE, 0.0},
    {HATCH_PICKUP, 180},
    {ROCKET_MID, -90.0},
    {CARGO_SIDE, 90.0},
    {ROCKET_CLOSE, -33.5},
    {ROCKET_FAR, -146.5}
  };

  enum DistanceType {
    LIDAR, ULTRASONIC, ENCODER, NAVX
  };

	unordered_map<Robot::DistanceType, std::string> distanceTypeNames = {
		{LIDAR, "LIDAR"},
		{ULTRASONIC, "ULTRASONIC"},
		{ENCODER, "ENCODER"},
		{NAVX, "NAVX"}
	};

  bool driveInverted = false;

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  #if LIMELIGHT_EXIST 
    std::shared_ptr<NetworkTable> limelight = inst.GetTable("limelight");
  #endif
  std::shared_ptr<NetworkTable> dash = inst.GetTable("SmartDashboard");

  double left_command  = 0.0;
  double right_command = 0.0;
  double toleranceScalar;

  // # OFFBOARD #
  frc::Joystick leftJoystick  = frc::Joystick(usb0);
  frc::Joystick rightJoystick = frc::Joystick(usb1);
  frc::Joystick gamerJoystick = frc::Joystick(usb2);
  frc::Joystick buttonJoystick = frc::Joystick(3);

  // # ONBOARD #
  // SENSORS
  // Lift stage mag sensors
  #if ELEVATOR_SENSOR_EXIST
    DIO* elevatorZero;
    bool elevatorAtZero = false;
  #endif
  int elevatorCalibratingLoopCount = 0;

/* deprecated
  DIO* liftBottom;
  DIO* liftMid;
  DIO* liftTop;
*/
  
/* Line following sensors - experimental
  DIO* lineSensorLeft;
  DIO* lineSensorMid;
  DIO* lineSensorRight;
  frc::AnalogInput* lineSensorLeft = new frc::AnalogInput(0);
  frc::AnalogInput* lineSensorMid = new frc::AnalogInput(1);
  frc::AnalogInput* lineSensor2 = new frc::AnalogInput(2);
  frc::AnalogInput* lineSensorRight = new frc::AnalogInput(3);
  frc::AnalogInput* lineSensornavx4 = new frc::AnalogInput(4);
*/

  #if LIDAR_EXIST // create lidar objects
    Lidar* leftLidar = new Lidar(LEFT_LIDAR_PORT);
    Lidar* rightLidar = new Lidar(RIGHT_LIDAR_PORT /* maybe need to put in address */);
  #endif

  #if ULTRA_EXIST // create ultrasonic objects
    /* plugging ultrasonics into other types of ports - experimental
      frc::Ultrasonic* leftUltrasonic = new frc::Ultrasonic(LEFT_ULTRASONIC_PING_CHANNEL, LEFT_ULTRASONIC_ECHO_CHANNEL);
      frc::Ultrasonic* rightUltrasonic = new frc::Ultrasonic(RIGHT_ULTRASONIC_PING_CHANNEL, RIGHT_ULTRASONIC_ECHO_CHANNEL);
      DIO* leftDioUltrasonic;
      DIO* rightDioUltrasonic;
      frc::SerialPort* serialUltrasonic = new frc::SerialPort(9600);
    */
    frc::AnalogInput* analogUltrasonicR = new frc::AnalogInput(ULTRASONIC_R_ANALOG_IN);
    frc::AnalogInput* analogUltrasonicL = new frc::AnalogInput(ULTRASONIC_L_ANALOG_IN);
  #endif

  struct distances {
    double left;
    double right;
    double total;
    double average() {
      return (left + right) / 2.0;
    }
  } ultraDist, lidarDist, limelightDist;


  // MOTOR CONTROLLERS
  // Talons

  // create motor controllers
  #if (COMP_ROBOT || PRACTICE_TALON)
    WPI_TalonSRX l1{LEFT_1_CAN_ADDRESS};
    WPI_VictorSPX l2{LEFT_2_CAN_ADDRESS};
    WPI_TalonSRX r1{RIGHT_1_CAN_ADDRESS};
    WPI_VictorSPX r2{RIGHT_2_CAN_ADDRESS};
  #else
    frc::Spark Spark_l1{L1_SPARK_PWM};
    frc::Spark Spark_l2{L2_SPARK_PWM};
    frc::Spark Spark_r1{R1_SPARK_PWM};
    frc::Spark Spark_r2{R2_SPARK_PWM};
  #endif

  frc::Spark intakeMotor{INTAKE_MOTOR_PWM};
  WPI_TalonSRX elevator{ELEVATOR_MOTOR_CAN_ADDRESS};

  // frc::DifferentialDrive drive{l1, r1};

  // OTHER

  AHRS* ahrs = new AHRS(I2C::Port::kMXP);
  frc::Timer* timer = new frc::Timer();

  frc::Compressor compressor{pcm0};
  #if COMP_ROBOT
    #if PNEUMATIC_OBJECT // suspect
      Pneumatic driveGearboxes{pcm0, pch0, pch1, frc::DoubleSolenoid::kReverse};
      Pneumatic intakeArm{     pcm0, pch6, pch7, frc::DoubleSolenoid::kReverse};
      Pneumatic ballPusher{    pcm0, pch2, pch3, frc::DoubleSolenoid::kReverse};
      Pneumatic hatchPusher{   pcm0, pch4, pch5, frc::DoubleSolenoid::kForward};
      Pneumatic jumper{           1,    0,    1, frc::DoubleSolenoid::kForward};
    #else
      frc::DoubleSolenoid driveGearboxes{pcm0, pch0, pch1};
      frc::DoubleSolenoid intakeArm{     pcm0, pch6, pch7};
      frc::DoubleSolenoid ballPusher{    pcm0, pch2, pch3};
      frc::DoubleSolenoid hatchPusher{   pcm0, pch4, pch5};
      frc::DoubleSolenoid jumper{           1,    0,    1};
    #endif
  #else // TODOOO
    #if PNEUMATIC_OBJECT
      Pneumatic driveGearboxes{pcm0, pch0, pch1, frc::DoubleSolenoid::kReverse};
      Pneumatic intakeArm{     pcm0, pch6, pch7, frc::DoubleSolenoid::kReverse};
      Pneumatic ballPusher{    pcm0, pch2, pch3, frc::DoubleSolenoid::kForward}; // good?
      Pneumatic hatchPusher{   pcm0, pch4, pch5, frc::DoubleSolenoid::kForward}; // good?
      Pneumatic jumper{           1,    0,    1, frc::DoubleSolenoid::kForward};
    #else
      frc::DoubleSolenoid driveGearboxes{pcm0, pch0, pch1};
      frc::DoubleSolenoid intakeArm{     pcm0, pch6, pch7};
      frc::DoubleSolenoid ballPusher{    pcm0, pch2, pch3};
      frc::DoubleSolenoid hatchPusher{   pcm0, pch4, pch5};
      frc::DoubleSolenoid jumper{           1,    0,    1};
    #endif
  #endif

  // frc::PIDController straighten = frc::PIDController();

  #if COMP_ROBOT
    frc::Encoder leftEnc{2, 3}, rightEnc{0, 1}; // might need to switch. also, TODO - Talon encoders?
  #else
    frc::Encoder leftEnc{2, 3}, rightEnc{0, 1}; // also TODO
  #endif

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledPeriodic() override;

  void TalonInit();
  void LineSensorInit();

  void ShiftGears(Robot::Direction dir, frc::DoubleSolenoid& solenoid);
  void ShiftGears(frc::DoubleSolenoid::Value state, frc::DoubleSolenoid& solenoid);
  void ShiftGears(bool upBtn, bool downBtn, frc::DoubleSolenoid& solenoid);
  void ToggleSolenoid(bool btn, frc::DoubleSolenoid& solenoid);
  void ToggleSolenoid(frc::DoubleSolenoid& Solenoid);

  void GetDistances();
  void Align(int left, int right, int tolerance, Robot::DistanceType type);
  void Align(Robot::DistanceType type);
  void LidarInit();
  void GetLimelight();

  void DriveLeft(double amount);
  void DriveRight(double amount);
  void TurnLeft(double amount);
  void TurnRight(double amount);
  void DriveBoth(double amount);
  void HandleJoysticks(); // TODO: better name
  void RunElevator();
  void RunElevator2();
  void ChooseElevatorMode();
  void ChooseAlignMode();

  void StopForwardMovement();

  void SetPneumaticDefaultDirections();
  void SensorInit();
  void UpdatePneumatics();

  // Utils:
  double calculateDampenedJoystick(double rawAxisValue);
  double linearMap(double n, double start1, double stop1, double start2, double stop2);

  void Testing();  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};