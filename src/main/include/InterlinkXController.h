#pragma once
#include "frc/Joystick.h"
#include "Vector.h"
#include <frc/smartdashboard/SmartDashboard.h>

class InterlinkXController
{
private:
    frc::Joystick *interlink;
    double x, y, z;
    double deadband = 0.06;
    Vector *fieldVelocitySetpoint;

public:
    // flight simulator joystick
    InterlinkXController(frc::Joystick *interlink)
    {
        this->interlink = interlink;
        fieldVelocitySetpoint = new Vector();
    }


    /**
     * returns a vector using right joystick of the controller
     * for controlling the field speed of the robot
     * */
    Vector *getFieldVelocitySetpoint() {
        x = 0;
        if (std::abs(1.205 * (interlink->GetRawAxis(0) + 0.01)) > deadband)
        {
            x = 1.205 * (interlink->GetRawAxis(0) + 0.01);
        }
        y = 0;
        if (std::abs(-1.3 * (interlink->GetRawAxis(1) - 0.03)) > deadband)
        {
            y = -1.3 * (interlink->GetRawAxis(1) - 0.03);
        }
        fieldVelocitySetpoint->set(x, y);
        return fieldVelocitySetpoint;
    }


    // gets the controller z-axis value, the left joystick horizontal movement
    double getZ()
    {
        z = 0;
        if (std::abs(1.198 * (interlink->GetRawAxis(5) + 0.08)) > deadband)
        {
            z = 1.198 * (interlink->GetRawAxis(5) + 0.08);
        }
        return z;
    }
};