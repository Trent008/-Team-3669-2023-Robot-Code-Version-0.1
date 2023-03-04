#pragma once
#include "Vector.h"

class ArmController
{
private:
    // axis limits
    const double j1_Min = 0;
    const double j1_Max = 14.25;
    const double j2_Min = -3;
    const double j2_Max = 20;
    double j4_Max;
    double j4_Min;
    // position setpoint limits
    const double deck = -38.75;
    const double frame = 26;
    const double floor = -45.5;
    Vector origin = {19.75, -49.5};
    Vector startPosition = {10.75, -39.75};
    Vector currentPosition;
    Vector targetPosition;
    Vector error;
    double positionRate, j3Rate, j4Rate;
    double j2Length;
    double startingJ1Length;
    double j1SetpointInches;
    double j3Current = 0;
    double j3Setpoint = 0;
    double j3Error;
    double j4UserSetpoint = 0;
    double j4Setpoint;
    double j4Current = 0;
    double startingJ4Position;
    double j4Error;

public:
    ArmController(double positionRate, double j3Rate, double j4Rate)
    {
        this->positionRate = positionRate;
        this->j3Rate = j3Rate;
        this->j4Rate = j4Rate;
        currentPosition = startPosition;
        targetPosition = startPosition;
        startingJ4Position = angle(startPosition) - 90;
        startingJ1Length = sqrt(356 - 4 * sqrt(7345) * cos(atan2(startPosition.getX(), -startPosition.getY()) + atan(1.5 / 15.5) - atan(3.0 / 11)));
        j4_Max = 90 - startingJ4Position;
        j4_Min = j4_Max - 180;
    }

    void update(Vector velocityTarget = {}, double j3Velocity = 0, double j4Velocity = 0)
    {

    /* ------------- limits the target position ------------- */
        targetPosition += velocityTarget;
        if ((targetPosition.getY() < deck) && (targetPosition.getY() > floor) && (targetPosition.getX() < frame))
        {
            if (std::abs(targetPosition.getY() - deck) < std::abs(targetPosition.getX() - frame))
            {
                targetPosition.setY(deck);
            }
            else
            {
                targetPosition.setX(frame);
            }
        }
        else if ((targetPosition.getY() < floor) && (targetPosition.getX() > frame))
        {
            targetPosition.setY(floor);
        }
        else if ((targetPosition.getY() <= floor) && (targetPosition.getX() <= frame))
        {
            targetPosition = Vector{frame, floor};
        }

    /* -- moves the currentPosition toward the target currentPosition -- */
        error = targetPosition - currentPosition;
        if (error > 2 * positionRate)
        {
            currentPosition += error * positionRate / abs(error);
        }
        else
        {
            currentPosition += error * 0.5;
        }

    /* ------------- limits the current position ------------- */
        if ((currentPosition.getY() < deck) && (currentPosition.getY() > floor) && (currentPosition.getX() < frame))
        {
            if (std::abs(currentPosition.getY() - deck) < std::abs(currentPosition.getX() - frame))
            {
                currentPosition.setY(deck);
            }
            else
            {
                currentPosition.setX(frame);
            }
        }
        else if ((currentPosition.getY() < floor) && (currentPosition.getX() > frame))
        {
            currentPosition.setY(floor);
        }
        else if ((currentPosition.getY() <= floor) && (currentPosition.getX() <= frame))
        {
            currentPosition = Vector{frame, floor};
        }

    /* ------------- set j4 position ------------- */
        j4UserSetpoint += j4Velocity;
        j4Setpoint = angle(currentPosition) - 90 - startingJ4Position + j4UserSetpoint;
        if (j4Setpoint > j4_Max)
        {
            j4UserSetpoint -= j4Setpoint - j4_Max;
            j4Setpoint = j4_Max;
        }
        if (j4Setpoint < j4_Min)
        {
            j4UserSetpoint += j4_Min - j4Setpoint;
            j4Setpoint = j4_Min;
        }

    /* --------- integrate toward new j4 position --------- */
        j4Error = j4Setpoint - j4Current;
        if (std::abs(j4Error) > 2 * j4Rate) {
            j4Current += j4Rate * j4Error / std::abs(j4Error);
        }
        else {
            j4Current += j4Error / 2;
        }

    /* ------------- set j3 position ------------- */
        j3Setpoint += j3Velocity;
        if (currentPosition.getX() < 26) { j3Setpoint = 0; }
        if (j3Setpoint > 90) { j3Setpoint = 90; }
        if (j3Setpoint < -90) { j3Setpoint = -90; }

    /* ------------- integrate toward new j3 position ------------- */
        j3Error = j3Setpoint - j3Current;
        if (std::abs(j3Error) > j3Rate) {
            j3Current += j3Rate * j3Error / std::abs(j3Error);
        }
        else {
            j3Current += j3Error / 2;
        }

    }

    double getJ1PIDReference()
    {
        j1SetpointInches = sqrt(356 - 4 * sqrt(7345) * cos(atan2(currentPosition.getX(), -currentPosition.getY()) + atan(1.5 / 15.5) - atan(3.0 / 11))) - startingJ1Length;
        j1SetpointInches = (j1SetpointInches > j1_Max) ? j1_Max : (j1SetpointInches < j1_Min) ? j1_Min : j1SetpointInches;
        return j1SetpointInches * (50.0 / 7);
    }

    double getJ2PIDReference()
    {
        j2Length = abs(currentPosition) - abs(startPosition);
        j2Length = (j2Length > j2_Max) ? j2_Max : (j2Length < j2_Min) ? j2_Min : j2Length;
        return j2Length * (5 * 25.4 / 18);
    }

    double getj3PIDReference()
    {
        return j3Current * (5.0 / 36);
    }

    double getJ4PIDReference()
    {
        return j4Current * (5.0 / 36);
    }

    void setArmPosition(Vector targetFromOrigin, double j4UserSetpoint, double j3Setpoint = 0)
    {
        targetPosition = origin + targetFromOrigin;
        this->j4UserSetpoint = j4UserSetpoint;
        this->j3Setpoint = j3Setpoint;
    }

    bool poseReached(double tolerance) {
        return error < tolerance;
    }

    double pumpPercent(bool pressure)
    {
        return ((pressure) ? 0.5 : 0);
    }
};
