#pragma once
#include "SwerveDrive.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of target positions and angles
 **/
class RobotPoseTargeting
{
    private:
        double positionProportional; // rate at which to approach the target position
        double angleProportional;   // rate at which to approach the target angle
        Pose poseError;      // how fast the robot needs to move to get to its next position setpoint
        Pose distanceToSetpointPose;
        Pose swerveRate;
        Pose target;
        SwerveDrive *swerve;

    public:
        RobotPoseTargeting(SwerveDrive *swerve, double positionProportional, double angleProportional) {
            this->positionProportional = positionProportional;
            this->angleProportional = angleProportional;
            this->swerve = swerve;
        }

        void targetPose(Pose setpoint, double driveRate, double rotationRate) {
            target.moveTowardPose(setpoint, driveRate / 50, rotationRate / 50);
            distanceToSetpointPose = setpoint - target;
            poseError = target - swerve->getPose();
            swerveRate = poseError * Vector{positionProportional, angleProportional};
            swerveRate.limit(Vector{0.5, 0.5});
            swerve->Set(swerveRate);
        }

        bool poseReached(double positionTolerance, double angleTolerance) {
            return (distanceToSetpointPose.getPosition() < positionTolerance) && (std::abs(distanceToSetpointPose.getAngle()) <= angleTolerance);
        }
};