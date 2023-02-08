// Field Oriented Control and Motion Smoothing
#pragma once
#include "AHRS.h"
#include "Vector.h"
#include <frc/smartdashboard/SmartDashboard.h>
/**
 * Field Oriented Control:
 * converts field velocity input to velocity relative to the robot
 * after smoothing the field velocity input
 * */
class FOC 

{
private:
    Vector *setpointFieldVelocity; // field velocity from the teleop/auto controller
    Vector *fieldVelocity;  // smoothed/accellerated field velocity
    Vector *fieldVelocityError; // difference between the current field velocity and the setpoint
    Vector *robotVelocity; // field re-oriented velocity
    Vector *fieldPosition; // robot's position on the field
    double rotationRate; // current rotation rate
    double rotationRateSetpoint; // scaled rotation rate setpoint
    double navXAngle;   // angle reported from the NavX2
    double rotationalAccelleration; // rate to accelerate the rotation rate input
    double velocityAccelleration;   // rate to accelerate the velocity input
    double drivingSpeed;    // how to scale the field velocity input
    double steeringSpeed;   // how to scale the robot rotation rate input
    AHRS navx{frc::SPI::Port::kMXP}; // NavX V2 object

public:
    FOC(double velocityAcceleration, double rotationalAccelleration, double drivingSpeed, double steeringSpeed)
    {
        setpointFieldVelocity = new Vector();
        fieldVelocity = new Vector();
        fieldVelocityError = new Vector();
        robotVelocity = new Vector();
        fieldPosition = new Vector();
        rotationRate = 0;
        this->velocityAccelleration = velocityAcceleration;
        this->rotationalAccelleration = rotationalAccelleration;
        this->drivingSpeed = drivingSpeed;
        this->steeringSpeed = steeringSpeed;
    }

    /**
     *  sets the field oriented and smoothed x velocity, 
     *  y velocity, and rotation rate for the robot
     * */
    void update(Vector *setpointFieldVelocity, double rotationRateSetpoint)
    {
        navXAngle = navx.GetYaw();
        /**--------------Field Velocity Accelleration--------------**/
        this->setpointFieldVelocity->set(setpointFieldVelocity);
        this->setpointFieldVelocity->scale(drivingSpeed);
        fieldVelocityError->set(this->setpointFieldVelocity);
        fieldVelocityError->subtractVector(fieldVelocity);
        if (fieldVelocityError->getMagnitude() > 0.04) // prevents divide by zero errors, and can prevent jitter
        {
            fieldVelocityError->scale(velocityAccelleration / fieldVelocityError->getMagnitude());
            fieldVelocity->addVector(fieldVelocityError);
        }
        /**-----------------Rotation Rate Accelleration-----------------**/
        this->rotationRateSetpoint = rotationRateSetpoint * steeringSpeed;
        if (std::abs(this->rotationRateSetpoint - rotationRate) > 0.04) // prevents divide by zero errors, and can prevent jitter
        {
            rotationRate += rotationalAccelleration * (this->rotationRateSetpoint - rotationRate)/std::abs(this->rotationRateSetpoint - rotationRate);
        }
        /**------------Field Oriented Control------------**/
        robotVelocity->set(fieldVelocity);
        robotVelocity->rotate(-navXAngle);
    }
    
    /**
     * Returns:
     * NavX2 yaw angle
     * -180 - 180 degrees
     * */
    double getRobotAngle()
    {
        return navXAngle;
    }

    /**
     * Returns:
     * field reoriented robot velocity
     * */
    Vector *getRobotVelocity(){
        return robotVelocity;
    }

    /**
     * Returns:
     * smoothed/accellerated robot rotation rate
     * */
    double getRotationRate() {
        return rotationRate;
    }
};