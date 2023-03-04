// Field Oriented Control and Motion Smoothing
#pragma once
#include "AHRS.h"
#include "Pose.h"

/**
 * Field Oriented Control:
 * converts field velocity input to velocity relative to the robot
 * after smoothing the field velocity input
 * */
class FOC

{
private:
    Pose fieldVelocity;            // smoothed/accellerated field velocity
    Vector robotVelocity;            // field re-oriented velocity
    double navXAngle;                // angle reported from the NavX2
    double rotationalAccelleration;  // rate to accelerate the rotation rate input
    double velocityAccelleration;    // rate to accelerate the velocity input
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
    Pose getRobotPoseVelocity(Pose velocitySetpoint)
    {
        navXAngle = navx.GetYaw();
        fieldVelocity.moveTowardPose(velocitySetpoint, velocityAccelleration, rotationalAccelleration);
        
        /**------------Field Oriented Control------------**/
        robotVelocity = fieldVelocity.getPosition();
        robotVelocity.rotate(-navXAngle);
        return Pose{robotVelocity, fieldVelocity.getAngle()};
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

    void zeroYaw()
    {
        navx.ZeroYaw();
    }
};