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

  driveMotor1.SetSelectedSensorPosition(0);
  driveMotor2.SetSelectedSensorPosition(0);
  driveMotor3.SetSelectedSensorPosition(0);
  driveMotor4.SetSelectedSensorPosition(0);
  drivingSpaceMouse.initialize(20);
}

void Robot::TeleopPeriodic() {
    drivingSpaceMouse.update();
    motionController.update(Vector{sM.x(), sM.y()}, sM.zr());
    swerve.run();

    //extensionSetpoint += 0.2*interLink.getX();
    //armExtenderPID.SetReference(extensionSetpoint, rev::CANSparkMax::ControlType::kPosition);
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
