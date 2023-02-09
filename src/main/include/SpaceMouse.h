#pragma once
#include "frc/Joystick.h"
#include "Vector.h"
#include <frc/smartdashboard/SmartDashboard.h>

class SpaceMouse
{
private:
    frc::Joystick *joy;
    Vector *velocity;
    Vector *lastPosition;
    Vector *position;
    double change;
    double lastValue;
    double value;

public:
    // SpaceMouse as joystick
    SpaceMouse(frc::Joystick *joy)
    {
        this->joy = joy;
        velocity = new Vector();
        position = new Vector();
        lastPosition = new Vector();
    }

    /**
     * returns a vector using right joystick of the controller
     * for controlling the field position of the robot
     * */
    Vector *getFieldVelocity() {
        position->set(joy->GetX(), -joy->GetY());
        velocity->set(position);
        velocity->subtractVector(lastPosition);
        lastPosition->set(position);
        return velocity;
    }

    // gets the controller z-axis value, the left joystick horizontal movement
    double getZ()
    {
        value = joy->GetRawAxis(5);
        change = value - lastValue;
        lastValue = value;
        return change;
    }


    Vector *getArmVelocity() {
        position->set(-joy->GetY(), -joy->GetZ());
        velocity->set(position);
        velocity->subtractVector(lastPosition);
        lastPosition->set(position);
        return velocity;
    }
    
    double getArmTwist() {
        return -joy->GetRawAxis(4);
    }

    double getArmWrist() {
        value = joy->GetRawAxis(3);
        change = value - lastValue;
        lastValue = value;
        return change;
    }
};