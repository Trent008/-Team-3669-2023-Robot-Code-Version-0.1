#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <InterlinkXController.h>
#include "LogitechJoystick.h"
#include "FOC.h"
#include "SwerveModule.h"
#include "SwerveDrive.h"
#include <frc/smartdashboard/SendableChooser.h>
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
  double robotAccel = 0.05;      // acceleration rate of the robot speed on the field
  double robotTurnAccel = 0.05; // acceleration rate of robot steering rate
  double robotSpeed = 0.6;      // maximum speed of field travel in the x or y direction - do not set > 1
  double robotTurnSpeed = 0.3;  // maximum turning speed - do not set > 1
  double moduleSteeringMotorP = 1;
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
  
  Vector *positionSetpoint;
  Vector *currentPosition;
  Vector *fieldVelocitySetpoint;
  
  double angleSetpoint;
  double currentAngle;
  double rotationRateSetpoint;


  frc::Joystick controller{0};       // does the necessary adjustments for the flight simulator controller and creates a field
  InterlinkXController *teleopController = new InterlinkXController(&controller); // velocity vector for the FOC update method
  // swerve module drive motors:
  WPI_TalonFX driveMotor1{11};
  WPI_TalonFX driveMotor2{12};
  WPI_TalonFX driveMotor3{13};
  WPI_TalonFX driveMotor4{14};
  // swerve module wheel orientation encoders:
  CANCoder encoder1{21};
  CANCoder encoder2{22};
  CANCoder encoder3{23};
  CANCoder encoder4{24};
  // swerve module wheel pivoting motors:
  rev::CANSparkMax steeringMotor1{31, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor2{32, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor3{33, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor4{34, rev::CANSparkMax::MotorType::kBrushless};
  // swerve module objects:
  SwerveModule *m1 = new SwerveModule{&driveMotor1, &steeringMotor1, &encoder1, -1, 1, moduleSteeringMotorP};
  SwerveModule *m2 = new SwerveModule{&driveMotor2, &steeringMotor2, &encoder2, -1, -1, moduleSteeringMotorP};
  SwerveModule *m3 = new SwerveModule{&driveMotor3, &steeringMotor3, &encoder3, 1, 1, moduleSteeringMotorP};
  SwerveModule *m4 = new SwerveModule{&driveMotor4, &steeringMotor4, &encoder4, 1, -1, moduleSteeringMotorP};
  // swerve module array:
  SwerveModule *modules[4] = {m1, m2, m3, m4};
  
  // field oriented motion control and motion smoothing class:
  FOC motionController{robotAccel, robotTurnAccel, robotSpeed, robotTurnSpeed};
  // swerve drive object to control the 4-SwerveModule array using the motion controller object
  SwerveDrive swerve{&motionController, modules};
  PositionAndAngleTargeting autonomousTargeting{0.04, 0.017, 1.0, 1.0};

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

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::Timer auto_timer;
};
