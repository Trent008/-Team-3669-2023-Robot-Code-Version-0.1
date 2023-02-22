// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  // swerve motor config
  driveMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
  driveMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
  driveMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);
  driveMotor4.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 20);

  // arm motor config
  left_leadscrew.SetP(0.1);
  right_leadscrew.SetP(0.1);
  armExtenderPID.SetP(0.1);
  wristPID.SetP(0.1);
  twistPID.SetP(0.1);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);

}

void Robot::AutonomousPeriodic() {
  currentPosition = swerve.getPosition();
  positionSetpoint = setpointList.getPosition(setpointIndex);
  fieldVelocitySetpoint = autonomousTargeting.targetPosition(currentPosition, positionSetpoint);
  
  angleSetpoint = setpointList.getAngle(setpointIndex);
  currentAngle = motionController.getRobotAngle();
  rotationRateSetpoint = autonomousTargeting.targetAngle(currentAngle, angleSetpoint);
  
  motionController.update(fieldVelocitySetpoint, rotationRateSetpoint);
  swerve.run();
  setpointIndex++;
}

void Robot::TeleopInit() {
  SMPro.initialize();
}

void Robot::TeleopPeriodic() {
    if (xboxC.getAPressed()) {motionController.zeroYaw();}

    SMPro.update();
    motionController.update(Vector{xboxC.getX(), xboxC.getY()}, xboxC.getZR());
    swerve.run();

    arm.update(Vector{SMPro.getY(), SMPro.getZ()}, SMPro.getYR(), SMPro.getXR());
    pump1.Set(arm.pumpPercent(pressure1.Get()));
    pump2.Set(arm.pumpPercent(pressure2.Get()));
    isHoldingCone = (SMPro.getCTRLPressed()) ? !isHoldingCone : isHoldingCone;
    suctionCup1.Set(isHoldingCone);
    suctionCup2.Set(isHoldingCone);
     armExtenderPID.SetReference(arm.getArmExtension(), rev::CANSparkMax::ControlType::kPosition);
     left_leadscrew.SetReference(arm.getLeadscrewExtension(), rev::CANSparkMax::ControlType::kPosition);
     right_leadscrew.SetReference(arm.getLeadscrewExtension(), rev::CANSparkMax::ControlType::kPosition);
     wristPID.SetReference(arm.getWristSetpoint(), rev::CANSparkMax::ControlType::kPosition);
     if (SMPro.get1Pressed()) {  arm.setArmPosition(Vector{10, 7}, -7);  }
     if (SMPro.getESCPressed()) {  arm.setArmPosition(Vector{-9, 13}, 10);  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
