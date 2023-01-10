#pragma once
#include "frc/Joystick.h"
#include "Utilities/Vector.h"
#include <frc/smartdashboard/SmartDashboard.h>

class LogitechJoystick
{
private:
    frc::Joystick *interlink;
    double x, y, z;
    double deadband = 0.001;
    Vector *fieldVelocitySetpoint;

public:
    // flight simulator joystick
    LogitechJoystick(frc::Joystick *interlink)
    {
        this->interlink = interlink;
        fieldVelocitySetpoint = new Vector(0, 0, false);
    }


    /**
     * returns a vector using right joystick of the controller
     * for controlling the field speed of the robot
     * */
    Vector *getFieldVelocitySetpoint() {
        x = 0;
        if (std::abs(1 * (interlink->GetX())) > deadband)
        {
            x = 1 * (interlink->GetX());
        }
        y = 0;
        if (std::abs(-1 * (interlink->GetY())) > deadband)
        {
            y = -1 * (interlink->GetY());
        }
        fieldVelocitySetpoint->set(x, y, 0);
        return fieldVelocitySetpoint;
    }


    // gets the controller z-axis value, the left joystick horizontal movement
    double getZ()
    {
        z = 0;
        if (std::abs(1 * (interlink->GetRawAxis(5))) > deadband)
        {
            z = 1 * (interlink->GetRawAxis(5));
        }
        return z;
    }
};