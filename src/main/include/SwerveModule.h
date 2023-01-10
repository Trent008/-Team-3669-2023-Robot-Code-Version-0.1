#pragma once
#include "ctre/phoenix.h"
#include "Utilities/Vector.h"
#include "AngleChooser.h"
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule
{
private:
    Vector *turnVector;     // vector corresponding to the way the rotation rate adds to the swerve module velocity
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
    Vector *wheelPositionChange;

public:
    /**
     * parameters posX and posY set the position of
     * the module relative to the center of the robot
     */
    SwerveModule(WPI_TalonFX* driveMotor, rev::CANSparkMax* steeringMotor, CANCoder* wheelEncoder, double posX, double posY, double steeringMotorP)
    {
        steeringMotorP = 1;
        this->driveMotor = driveMotor;
        this->steeringMotor = steeringMotor;
        this->wheelEncoder = wheelEncoder;
        this->steeringMotorP = steeringMotorP;
        turnVector = new Vector(posX, posY, 0);
        turnVector->rotate(90);
        wheelPositionChange = new Vector(0, 0, 1);
    }

    // returns the angle the wheel needs to turn to

    void findSpeedAndAngleError(Vector *velocity)
    {
        angleWheel = wheelEncoder->GetAbsolutePosition();
        angleSetpoint = velocity->getAngle();
        angleError = angleChooser.getShortestAngle(angleWheel, angleSetpoint);
        wheelDirection = angleChooser.getDirection();
        wheelSpeed = wheelDirection * velocity->getMagnitude();
    }

    void Set(Vector *velocity) {
        findSpeedAndAngleError(velocity);
        driveMotor->Set(wheelSpeed);
        steeringMotor->Set(angleError * (-steeringMotorP) / 180);
        currentPosition = driveMotor->GetSelectedSensorPosition(0);
        wheelPositionChange->set(wheelEncoder->GetAbsolutePosition(), currentPosition - lastPosition, true);
        lastPosition = currentPosition;
    }

    Vector *getTurnVector()
    {
        return turnVector;
    }

    Vector *getwheelPositionChange() {
        return wheelPositionChange;
    }
};