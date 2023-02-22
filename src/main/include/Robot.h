#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "frc/DigitalInput.h"
#include "frc/Solenoid.h"
#include "InterLinkX.h"
#include "SpaceMousePro.h"
#include "SpaceMouseEnt.h"
#include "XBOXController.h"
#include "FOC.h"
#include "SwerveModule.h"
#include "SwerveDrive.h"
#include "ArmController.h"
#include <frc/Joystick.h>
#include "ctre/phoenix.h"
#include <rev/CANSparkMax.h>
#include <math.h>
#include <cmath>
#include "PositionAndAngleTargeting.h"
#include "AutoSetpointList.h"

class Robot : public frc::TimedRobot
{

public:
  double deadband = 0.08;
  double robotAccel = 0.03;      // acceleration rate of the robot speed on the field
  double robotTurnAccel = 0.03; // acceleration rate of robot steering rate

  /**
   * the coordinates, angles, and wait times 
   * for the autonomous routine
   **/
  double autoSetpoints[10][4] =
  {
    {0,0,0,0},  // #1
    {0,78.5,90,2},  // #2
    {0,0,0,2},  // #3
    {0,0,0,0},  // #4
    {0,0,0,0},  // #5
    {0,0,0,0},  // #6
    {0,0,0,0},  // #7
    {0,0,0,0},  // #8
    {0,0,0,0},  // #9
    {0,0,0,0},  // #10
  };
  AutoSetpointList setpointList{autoSetpoints};
  int setpointIndex = 0;  // keeps track of the index of the point the robot needs to go to during the autonomous
  
  Vector positionSetpoint;
  Vector currentPosition;
  Vector fieldVelocitySetpoint;
  Vector spaceMouseSpeed;
  
  double angleSetpoint;
  double currentAngle;
  double rotationRateSetpoint;

  double extensionSetpoint;
  double currentExtension;
  double extensionRateSetpoint;


  frc::Joystick driveController{0};
  frc::Joystick armController{1};
  /* ------ driving controller types ------ */ 
  XBOXController xboxC{&driveController};
  InterLinkX interLink{&driveController};
  SpaceMouseEnt SMEnt{&driveController};

  /* -------- arm controller types -------- */ 
  SpaceMousePro SMPro{&armController, 20};

 /* -------- swerve drive motors -------- */ 
  WPI_TalonFX driveMotor1{11};
  WPI_TalonFX driveMotor2{12};
  WPI_TalonFX driveMotor3{13};
  WPI_TalonFX driveMotor4{14};
/* -------- swerve module encoders -------- */ 
  CANCoder encoder1{21};
  CANCoder encoder2{22};
  CANCoder encoder3{23};
  CANCoder encoder4{24};
/* -------- swerve module wheel turning motors -------- */ 
  rev::CANSparkMax steeringMotor1{31, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor2{32, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor3{33, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor4{34, rev::CANSparkMax::MotorType::kBrushless};
  /* -------- swerve module objects -------- */ 
  SwerveModule *m1 = new SwerveModule{&driveMotor1, &steeringMotor1, &encoder1, -.7, 1};
  SwerveModule *m2 = new SwerveModule{&driveMotor2, &steeringMotor2, &encoder2, -.7, -1};
  SwerveModule *m3 = new SwerveModule{&driveMotor3, &steeringMotor3, &encoder3, .7, 1};
  SwerveModule *m4 = new SwerveModule{&driveMotor4, &steeringMotor4, &encoder4, .7, -1};
  // swerve module array:
  SwerveModule *modules[4] = {m1, m2, m3, m4};
  
  // field oriented motion control and motion smoothing class:
  FOC motionController{robotAccel, robotTurnAccel};
  // swerve drive object to control the 4-SwerveModule array using the motion controller object
  SwerveDrive swerve{&motionController, modules};
  PositionAndAngleTargeting autonomousTargeting{0.04, 0.007, 1.0, 1.0};

  // leadscrew motors and PID controllers
  rev::CANSparkMax leftLeadscrewM{41, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightLeadscrewM{42, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController left_leadscrew = leftLeadscrewM.GetPIDController();
  rev::SparkMaxPIDController right_leadscrew = rightLeadscrewM.GetPIDController();

  // arm extension motor
  rev::CANSparkMax armExtender{43, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController armExtenderPID = armExtender.GetPIDController();
  
  // end-of-arm motors
  rev::CANSparkMax twistMotor{44, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController twistPID = twistMotor.GetPIDController();
  rev::CANSparkMax wristMotor{45, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController wristPID = wristMotor.GetPIDController();

  // vacuum pumps
  WPI_TalonSRX pump1{51};
  WPI_TalonSRX pump2{52};
  frc::DigitalInput pressure1{0};
  frc::DigitalInput pressure2{1};
  frc::Solenoid suctionCup1{frc::PneumaticsModuleType::REVPH, 0};
  frc::Solenoid suctionCup2{frc::PneumaticsModuleType::REVPH, 15};
  bool isHoldingCone = false;


  ArmController arm{0.4, 7};

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
};
