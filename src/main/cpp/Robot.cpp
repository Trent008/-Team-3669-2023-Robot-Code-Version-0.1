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
  if (setpointIndex < 750) {
    currentPosition = swerve.getPosition();
    positionSetpoint = setpointList.getPose(setpointIndex).getPosition();
    fieldVelocitySetpoint = autonomousTargeting.targetPosition(currentPosition, positionSetpoint);
    
    angleSetpoint = setpointList.getPose(setpointIndex).getAngle();
    currentAngle = motionController.getRobotAngle();
    rotationRateSetpoint = autonomousTargeting.targetAngle(currentAngle, angleSetpoint);
    
    motionController.update(fieldVelocitySetpoint, rotationRateSetpoint);
    swerve.run();
    pump1.Set(arm.pumpPercent(pressure1.Get()));
    pump2.Set(arm.pumpPercent(pressure2.Get()));
    isHoldingCone = setpointList.getPose(setpointIndex).getSuctionCupState();
    suctionCup1.Set(isHoldingCone);
    suctionCup2.Set(isHoldingCone);
    // delay to grab cone
    if (setpointIndex > 25) {
      // set arm PID references
      arm.setArmPosition(setpointList.getPose(setpointIndex).getArmPosition(), setpointList.getPose(setpointIndex).getWristAngle());
      arm.update();
    }
  }
  else {  // go to default position at the end of autonomous
    motionController.update(Vector{}, 0);
    swerve.run();

    left_J1_PID.SetReference(arm.getJ1PIDReference(), rev::CANSparkMax::ControlType::kPosition);
    right_J1_PID.SetReference(arm.getJ1PIDReference(), rev::CANSparkMax::ControlType::kPosition);
    j2_PID.SetReference(arm.getJ2PIDReference(), rev::CANSparkMax::ControlType::kPosition);
    j3_PID.SetReference(arm.getj3PIDReference(), rev::CANSparkMax::ControlType::kPosition);
    j4_PID.SetReference(arm.getJ4PIDReference(), rev::CANSparkMax::ControlType::kPosition);
    arm.setArmPosition(Vector{-9, 10}, 0);
    arm.update();
  }
  setpointIndex++;
}

void Robot::TeleopInit() {
  SMPro.initialize();
}

void Robot::TeleopPeriodic() {
  if (SMPro.getMenuPressed()) {motionController.zeroYaw();}

  SMPro.update();
  motionController.update(Vector{SMEnt.getX(), SMEnt.getY()}, SMEnt.GetZR());
  swerve.run();

  //pump1.Set(arm.pumpPercent(pressure1.Get()));
  //pump2.Set(arm.pumpPercent(pressure2.Get()));
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
  if (SMPro.getAltPressed()) {  arm.setArmPosition(Vector{10, 7}, -7, 0);  }
  if (SMPro.get1Pressed()) {arm.setArmPosition(Vector{20, 39}, 0, 0);}
  if (SMPro.get2Pressed()) {arm.setArmPosition(Vector{21, 43}, 0, 0);}
  if (SMPro.get3Pressed()) {arm.setArmPosition(Vector{37, 55}, 0, 0);}
  if (SMPro.getESCPressed()) {  
    isHoming = true;
    arm.setArmPosition(Vector{8, 14}, 10, 0);  
  }
  arm.update(Vector{SMPro.getY(), SMPro.getZ()}, SMPro.getYR(), SMPro.getXR());
  if (isHoming && arm.getArmError() < 1) {
    isHoming = false;
    arm.setArmPosition(Vector{-9, 9.75}, 10, 0);
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
