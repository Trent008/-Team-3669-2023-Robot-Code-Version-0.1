#pragma once
#include "ctre/phoenix.h"
#include "Vector.h"
#include "AngleChooser.h"
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule
{
private:
    Vector turnVector;     // vector corresponding to the way the rotation rate adds to the swerve module velocity
    double wheelDirection = 0;  // direction of the wheel depending on whether the wheel is drivinging forward or backward
    double wheelSpeed = 0;
    double angleError = 0;
    double steeringMotorP;  // proportional value determines how quickly the steering responds to angle setpoints
    double option[4];
    double angleWheel, angleSetpoint;
    double lastPosition = 0;
    double currentPosition;
    AngleChooser angleChooser{};
    WPI_TalonFX* driveMotor;
    rev::CANSparkMax* steeringMotor;
    CANCoder* wheelEncoder;
    Vector wheelPositionChange;

public:
    /**
     * parameters posX and posY set the position of
     * the module relative to the center of the robot
     */
    SwerveModule(WPI_TalonFX* driveMotor, rev::CANSparkMax* steeringMotor, CANCoder* wheelEncoder, double posX, double posY)
    {
        steeringMotorP = 1;
        this->driveMotor = driveMotor;
        this->steeringMotor = steeringMotor;
        this->wheelEncoder = wheelEncoder;
        this->steeringMotorP = steeringMotorP;
        turnVector = Vector{posX, posY};
        turnVector.rotate(90);
    }

    // returns the angle the wheel needs to turn to
    void findSpeedAndAngleError(Vector velocity)
    {
        angleWheel = wheelEncoder->GetAbsolutePosition();
        angleSetpoint = angle(velocity);
        angleError = angleChooser.getShortestAngle(angleWheel, angleSetpoint);
        wheelDirection = angleChooser.getDirection();
        wheelSpeed = wheelDirection * abs(velocity);
    }

    void Set(Vector velocity) {  // todo: change to velocity mode
        findSpeedAndAngleError(velocity);
        driveMotor->Set(wheelSpeed);
        steeringMotor->Set(angleError * (-steeringMotorP) / 180);
        currentPosition = driveMotor->GetSelectedSensorPosition(0);
        wheelPositionChange.setPolar(currentPosition - lastPosition, wheelEncoder->GetAbsolutePosition());
        lastPosition = currentPosition;
    }

    Vector getTurnVector()
    {
        return turnVector;
    }

    double getWheelSpeed() {
        return wheelSpeed;
    }

    Vector getwheelPositionChange() {
        return wheelPositionChange;
    }
};