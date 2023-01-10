#pragma once
#include "Utilities/Vector.h"
#include "AngleChooser.h"
#include "frc/smartdashboard/SmartDashboard.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of target positions and angles
 **/
class PositionAndAngleTargeting
{
    private:
        double positionProportional; // rate at which to approach the target position
        double angleProportional;   // rate at which to approach the target angle
        double maxDriveRate;
        double maxRotationRate;
        Vector *positionError;      // how fast the robot needs to move to get to its next position setpoint
        AngleChooser angleChooser{};    // finds the most efficient way for the robot to turn to a given angle
        double angleError;          // how fast the robot needs to turn to get to its next angle setpoint
        Vector *distanceFromPositionTarget; // how far the robot is from its next position setpoint
        double distanceFromAngleTarget; // how far the robot is from its next angle setpoint

    public:
        PositionAndAngleTargeting(double positionProportional, double angleProportional, double maxDriveRate, double maxRotationRate) {
            this->positionProportional = positionProportional;
            this->angleProportional = angleProportional;
            this->maxDriveRate = maxDriveRate;
            this->maxRotationRate = maxRotationRate;
            positionError = new Vector(0, 0, true);
            distanceFromPositionTarget = new Vector(0, 0, true);
        }

        /**
         * Returns:
         * velocity vector in the direction of the next position setpoint
         * */
        Vector *targetPosition(Vector *currentPosition, Vector *targetPosition) {
            distanceFromPositionTarget->set(targetPosition);
            distanceFromPositionTarget->subtractVector(currentPosition);    // find the difference betweent the target position and the current position
            positionError->set(distanceFromPositionTarget);     
            positionError->scale(positionProportional);     // multiply the difference by the proportional value
            if(positionError->getMagnitude() > maxDriveRate) {
                positionError->scale(maxDriveRate / positionError->getMagnitude());     // limit the difference to the maxSpeed value
            }
            return positionError;
        }

        /**
         * Returns:
         * rotation rate required to turn the robot to the next angle setpoint
         **/
        double targetAngle(double currentAngle, double setpointAngle) {
            distanceFromAngleTarget = angleChooser.getShortestDirection(currentAngle, setpointAngle); // find the difference betweent the target angle and the current angle
            angleError = distanceFromAngleTarget;
            angleError *= angleProportional;        // multiply the difference by the proportional value
            if(std::abs(angleError) > maxRotationRate) {    
                angleError *= maxRotationRate / std::abs(angleError); // limit the difference to the max rotation rate value
            }
            return angleError;
            
        }
};