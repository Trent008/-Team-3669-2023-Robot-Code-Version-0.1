#pragma once
#include "Vector.h"
#include "cmath"
#include "math.h"

class ArmController
{
private:
    double minExtension = -3;
    double maxExtension = 20;
    double minLeadscrew = 0;
    double maxLeadscrew = 14;
    Vector originFromP = Vector{20.5, -49.5};
    Vector startPositionFromO = Vector{-9.25, 10.25};
    Vector startPositionFromP = Vector{11.25, -39.25};
    Vector currentPositionFromP;
    Vector targetPositionFromP;
    Vector error;
    double speed;
    double wristSpeed;
    double extenderIPR = 18 / 25.4 / 5; // inches of extension per revolution of the motor
    double extenderRPI = 5 * 25.4 / 18; // Revolutions of the motor needed for one inch of extension
    double leadscrewIPR = 7.0 / 50;
    double leadscrewRPI = 50.0 / 7;
    double armLength;
    double leadscrewLength;


    double startingWristPosition;
    double maxWristSetpoint;
    double minWristSetpoint;
    double wristPosition;
    double twistPosition;
    double wristSetpoint;
    double deck = -38;
    double frame = 26;
    double floor = -45.5;

public:
    ArmController(double speed, double wristSpeed)
    {
        this->speed = speed;
        this->wristSpeed = wristSpeed;
        currentPositionFromP = startPositionFromP;
        targetPositionFromP = startPositionFromP;
        startingWristPosition = angle(startPositionFromP) - 90;
        maxWristSetpoint = 90 - startingWristPosition;
        minWristSetpoint = maxWristSetpoint - 180;
        wristPosition = 0;
    }

    void update(Vector velocityTarget, double armTwist, double armWrist)
    {
        targetPositionFromP += velocityTarget;
        if ((targetPositionFromP.getY() < deck) && (targetPositionFromP.getY() > floor) && (targetPositionFromP.getX() < frame))
        {
            if (std::abs(targetPositionFromP.getY() - deck) < std::abs(targetPositionFromP.getX() - frame))
            {
                targetPositionFromP.setY(deck);
            }
            else
            {
                targetPositionFromP.setX(frame);
            }
        }
        else if ((targetPositionFromP.getY() < floor) && (targetPositionFromP.getX() > frame))
        {
            targetPositionFromP.setY(floor);
        }
        else if ((targetPositionFromP.getY() <= floor) && (targetPositionFromP.getX() <= frame))
        {
            targetPositionFromP = Vector{frame, floor};
        }
        // moves the currentPositionFromP toward the target currentPositionFromP
        error = targetPositionFromP - currentPositionFromP;
        if (abs(error) > 2 * speed)
        {
            error *= speed / abs(error);
        }
        else
        {
            error *= 0.5;
        }

        currentPositionFromP += error;
        if ((currentPositionFromP.getY() < deck) && (currentPositionFromP.getY() > floor) && (currentPositionFromP.getX() < frame))
        {
            if (std::abs(currentPositionFromP.getY() - deck) < std::abs(currentPositionFromP.getX() - frame))
            {
                currentPositionFromP.setY(deck);
            }
            else
            {
                currentPositionFromP.setX(frame);
            }
        }
        else if ((currentPositionFromP.getY() < floor) && (currentPositionFromP.getX() > frame))
        {
            currentPositionFromP.setY(floor);
        }
        else if ((currentPositionFromP.getY() <= floor) && (currentPositionFromP.getX() <= frame))
        {
            currentPositionFromP = Vector{frame, floor};
        }

        wristPosition += armWrist * wristSpeed;
        wristSetpoint = angle(currentPositionFromP) - 90 - startingWristPosition + wristPosition;
        if (wristSetpoint > maxWristSetpoint)
        {
            wristPosition -= wristSetpoint - maxWristSetpoint;
            wristSetpoint = maxWristSetpoint;
        }
        if (wristSetpoint < minWristSetpoint)
        {
            wristPosition += minWristSetpoint - wristSetpoint;
            wristSetpoint = minWristSetpoint;
        }
    }

    double getLeadscrewExtension()
    {
        leadscrewLength = sqrt(356 - 4 * sqrt(7345) * cos(atan2(currentPositionFromP.getX(), -currentPositionFromP.getY()) + atan(1.0 / 15.0) - atan(3.0 / 11.0))) 
                        - sqrt(356 - 4 * sqrt(7345) * cos(atan2(startPositionFromP.getX(), -startPositionFromP.getY()) + atan(1.0 / 15) - atan(3.0 / 11)));
        leadscrewLength = (leadscrewLength > maxLeadscrew) ? maxLeadscrew : (leadscrewLength < minLeadscrew) ? minLeadscrew
                                                                                                             : leadscrewLength;
        return leadscrewLength * leadscrewRPI;
    }

    double getArmExtension()
    {
        armLength = abs(currentPositionFromP) - abs(startPositionFromP);
        armLength = (armLength > maxExtension) ? maxExtension : (armLength < minExtension) ? minExtension
                                                                                           : armLength;
        return armLength * extenderRPI;
    }
    
    double getWristSetpoint()
    {
        return wristSetpoint * 50 / 360;
    }

    double getWristAngle() {
        return wristSetpoint;
    }

    Vector getPosition()
    {
        return currentPositionFromP;
    }

    void setArmPosition(Vector targetFromOrigin, double wristPosition)
    {
        targetPositionFromP = originFromP + targetFromOrigin;
        this->wristPosition = wristPosition;
    }

    double pumpPercent(bool pressure)
    {
        return ((pressure) ? 0.5 : 0);
    }
};
