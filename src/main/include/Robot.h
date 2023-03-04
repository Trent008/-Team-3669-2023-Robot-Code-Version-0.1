#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "frc/DigitalInput.h"
#include "frc/Solenoid.h"
#include "cameraserver/CameraServer.h"
#include "InterLinkX.h"
#include "SpaceMousePro.h"
#include "SpaceMouseEnt.h"
#include "XBOXController.h"
#include "ArmController.h"
#include "RobotPoseTargeting.h"
#include "AutoSetpoint.h"

class Robot : public frc::TimedRobot
{

public:
  double robotAccel = 0.03;     // acceleration rate of the robot speed on the field
  double robotTurnAccel = 0.03; // acceleration rate of robot steering rate
  Vector loadingStation = {20, 39};
  Vector cone1 = {21, 40};
  Vector cone2 = {37, 51};
  Vector cube1 = {13, 30};
  Vector cube2 = {30, 45};
  Vector pole1 = {74, 18.5};
  /*
   * the coordinates, angles, wait times, arm coordinates, suction
   * for the autonomous routine
   */
  AutoSetpoint setpoints[15] =
      {
          {Pose{}, {-9, 9.75}, true, 0},
          {Pose{{0, 5}, 0}, cone2, true, 0},
          {Pose{{0, 21.7}, 0}, cone2, true, 0},
          {Pose{{0, 21.7}, 0}, cone2 - Vector{0,4}, true, 0},
          {Pose{{0, 5}, 0}, pole1, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
          {Pose{}, {-9, 9.75}, false, 0},
      };

  int i = 0; // keeps track of the autonomous point index
  int t = 0; // keeps track of the number of processer

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
  SwerveModule *m1 = new SwerveModule{&driveMotor1, &steeringMotor1, &encoder1, {-.7, 1}};
  SwerveModule *m2 = new SwerveModule{&driveMotor2, &steeringMotor2, &encoder2, {-.7, -1}};
  SwerveModule *m3 = new SwerveModule{&driveMotor3, &steeringMotor3, &encoder3, {.7, 1}};
  SwerveModule *m4 = new SwerveModule{&driveMotor4, &steeringMotor4, &encoder4, {.7, -1}};
  // swerve module array:
  SwerveModule *modules[4] = {m1, m2, m3, m4};

  // field oriented motion control and motion smoothing class:
  FOC motionController{robotAccel, robotTurnAccel};
  // swerve drive object to control the 4-SwerveModule array using the motion controller object
  SwerveDrive swerve{&motionController, modules};
  RobotPoseTargeting swerveTargeting{&swerve, 0.04, 0.007};

  // leadscrew motors and PID controllers
  rev::CANSparkMax left_J1_NEO{41, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax right_J1_NEO{42, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController left_J1_PID = left_J1_NEO.GetPIDController();
  rev::SparkMaxPIDController right_J1_PID = right_J1_NEO.GetPIDController();

  // arm extension motor and PID controller
  rev::CANSparkMax j2_NEO{43, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController j2_PID = j2_NEO.GetPIDController();

  // end-of-arm motors and PID controllers
  rev::CANSparkMax j3_NEO{44, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController j3_PID = j3_NEO.GetPIDController();
  rev::CANSparkMax j4_NEO{45, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController j4_PID = j4_NEO.GetPIDController();

  // vacuum pumps
  WPI_TalonSRX pump1{51};
  WPI_TalonSRX pump2{52};
  frc::DigitalInput pressure1{0};
  frc::DigitalInput pressure2{1};
  frc::Solenoid suctionCup1{frc::PneumaticsModuleType::REVPH, 0};
  frc::Solenoid suctionCup2{frc::PneumaticsModuleType::REVPH, 15};
  bool isHoldingCone = false;
  bool isHoming = false;

  ArmController arm{0.8, 4, 4};

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
  static void VisionThread()
  {
    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(800, 600);
  }
};
