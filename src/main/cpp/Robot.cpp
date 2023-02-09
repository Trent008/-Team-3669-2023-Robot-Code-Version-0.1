// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  driveMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  driveMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  driveMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  driveMotor4.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  motionController.update(new Vector(), 0);
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);

}

void Robot::AutonomousPeriodic() {
  positionSetpoint = setpointList.getPosition(setpointIndex);
  currentPosition = swerve.getPosition();
  fieldVelocitySetpoint = autonomousTargeting.targetPosition(currentPosition, positionSetpoint);
  
  angleSetpoint = setpointList.getAngle(setpointIndex);
  currentAngle = motionController.getRobotAngle();
  rotationRateSetpoint = autonomousTargeting.targetAngle(currentAngle, angleSetpoint);
  
  motionController.update(fieldVelocitySetpoint, rotationRateSetpoint);
  swerve.run();
  setpointIndex++;
}

void Robot::TeleopInit() {
  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);

}

void Robot::TeleopPeriodic() {
    currentPosition = swerve.getPosition();
    positionSetpoint->addVector(drivingSpaceMouse.getFieldVelocity());
    fieldVelocitySetpoint = autonomousTargeting.targetPosition(currentPosition, positionSetpoint);
    
    currentAngle = motionController.getRobotAngle();
    angleSetpoint += drivingSpaceMouse.getZ();
    rotationRateSetpoint = autonomousTargeting.targetAngle(currentAngle, angleSetpoint);
    
    motionController.update(fieldVelocitySetpoint, rotationRateSetpoint);
    swerve.run();
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
