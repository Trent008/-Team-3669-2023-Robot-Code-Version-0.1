#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>



void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();

  // swerve motor config
  driveMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
  driveMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
  driveMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
  driveMotor4.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);

  // arm PID config
  left_J1_PID.SetP(0.1);
  right_J1_PID.SetP(0.1);
  j2_PID.SetP(0.1);
  j3_PID.SetP(0.1);
  j4_PID.SetP(0.1);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);
  motionController.zeroYaw();
}

void Robot::AutonomousPeriodic() {
  swerveTargeting.targetPose(setpoints[i].pose, setpoints[i].driveRate, setpoints[i].rotationRate);

  pump1.Set(arm.pumpPercent(pressure1.Get()));
  pump2.Set(arm.pumpPercent(pressure2.Get()));
  isHoldingCone = setpoints[i].suctionCupState;
  suctionCup1.Set(isHoldingCone);
  suctionCup2.Set(isHoldingCone);
  arm.setArmPosition(setpoints[i].armPosition, setpoints[i].wristAngle);
  arm.update();
  left_J1_PID.SetReference(arm.getJ1PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  right_J1_PID.SetReference(arm.getJ1PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  j2_PID.SetReference(arm.getJ2PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  j3_PID.SetReference(arm.getj3PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  j4_PID.SetReference(arm.getJ4PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  if (arm.poseReached(1) && swerveTargeting.poseReached(3, 5) && (i < 14)) {
    i++;
  }
  t++;
}

void Robot::TeleopInit() {
  SMPro.initialize();
}

void Robot::TeleopPeriodic() {
  if (xboxC.getAPressed()) {motionController.zeroYaw();}

  SMPro.update();
  swerve.Set(xboxC.getFieldPoseVelocity());

  pump1.Set(arm.pumpPercent(pressure1.Get()));
  pump2.Set(arm.pumpPercent(pressure2.Get()));
  isHoldingCone = (SMPro.getCTRLPressed()) ? !isHoldingCone : isHoldingCone;
  suctionCup1.Set(isHoldingCone);
  suctionCup2.Set(isHoldingCone);

  // set arm PID references
  left_J1_PID.SetReference(arm.getJ1PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  right_J1_PID.SetReference(arm.getJ1PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  j2_PID.SetReference(arm.getJ2PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  j3_PID.SetReference(arm.getj3PIDReference(), rev::CANSparkMax::ControlType::kPosition);
  j4_PID.SetReference(arm.getJ4PIDReference(), rev::CANSparkMax::ControlType::kPosition);

  // arm position buttons
  if (SMPro.getAltPressed()) {  arm.setArmPosition({10, 7}, -7, 0);  }
  if (SMPro.get1Pressed()) {arm.setArmPosition(cone1 + Vector{0, 3}, 0, 0);}
  if (SMPro.get2Pressed()) {arm.setArmPosition(cone2 + Vector{0, 3}, 0, 0);}
  if (SMPro.get3Pressed()) {arm.setArmPosition(cube1, 0, 0);}
  if (SMPro.get4Pressed()) {arm.setArmPosition(cube2, 0, 0);}
  if (SMPro.getESCPressed()) {  
    isHoming = true;
    arm.setArmPosition(Vector{8, 14}, 10, 0);  
  }
  arm.update(Vector{SMPro.getY(), SMPro.getZ()}, SMPro.getYR(), SMPro.getXR());
  if (isHoming && arm.poseReached(1)) {
    isHoming = false;
    arm.setArmPosition({-9, 9.75}, 10, 0);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
