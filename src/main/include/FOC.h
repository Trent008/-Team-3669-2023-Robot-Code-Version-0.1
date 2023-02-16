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
    Vector fieldVelocity;  // smoothed/accellerated field velocity
    Vector fieldVelocityError; // difference between the current field velocity and the setpoint
    Vector robotVelocity; // field re-oriented velocity
    double rotationRate = 0; // current rotation rate
    double navXAngle;   // angle reported from the NavX2
    double rotationalAccelleration; // rate to accelerate the rotation rate input
    double velocityAccelleration;   // rate to accelerate the velocity input
    AHRS navx{frc::SPI::Port::kMXP}; // NavX V2 object

public:
    FOC(double velocityAcceleration, double rotationalAccelleration)
    {
        this->velocityAccelleration = velocityAcceleration;
        this->rotationalAccelleration = rotationalAccelleration;
    }

    /**
     *  sets the field oriented and smoothed x velocity, 
     *  y velocity, and rotation rate for the robot
     * */
    void update(Vector setpointFieldVelocity, double rotationRateSetpoint)
    {
        navXAngle = navx.GetYaw();
        /**--------------Field Velocity Accelleration--------------**/
        fieldVelocityError = setpointFieldVelocity - fieldVelocity;
        if (abs(fieldVelocityError) > 2*velocityAccelleration) // prevents divide by zero errors and jitter
        {
            fieldVelocityError *= velocityAccelleration / abs(fieldVelocityError);
            fieldVelocity += fieldVelocityError;
        }
        else
        {
            fieldVelocityError /= 2;
            fieldVelocity += fieldVelocityError;
        }

        /**-----------------Rotation Rate Accelleration-----------------**/
        if (std::abs(rotationRateSetpoint - rotationRate) > 2.0*rotationalAccelleration) // prevents divide by zero errors and jitter
        {
            rotationRate += rotationalAccelleration * (rotationRateSetpoint - rotationRate)/std::abs(rotationRateSetpoint - rotationRate);
        }
        else
        {
            rotationRate += (rotationRateSetpoint - rotationRate)/2.0;
        }

        /**------------Field Oriented Control------------**/
        robotVelocity = fieldVelocity;
        robotVelocity.rotate(-navXAngle);
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
    Vector getRobotVelocity(){
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