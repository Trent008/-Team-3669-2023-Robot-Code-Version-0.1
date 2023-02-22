#pragma once
#include "frc/Joystick.h"

class XBOXController
{
private:
    frc::Joystick *joy;
    double nominalSpeed = 0.1;
    double deadband = 0.03;
    double a;

public:
    XBOXController(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    double getSpeed()
    {
        return nominalSpeed + joy->GetRawAxis(3) * (1 - nominalSpeed);
    }

    double getX()
    {
        a = getSpeed() * joy->GetRawAxis(0);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);

        return a;
    }

    double getY()
    {
        a = getSpeed() * -joy->GetRawAxis(1);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);
        return a;
    }

    // get Z Rotate axis
    double getZR()
    {
        a = getSpeed() * joy->GetRawAxis(4);
        if (!std::signbit(a))
        {
            a -= deadband;
            if (std::signbit(a))
            {
                a = 0;
            }
        }
        else if (std::signbit(a))
        {
            a += deadband;
            if (!std::signbit(a))
            {
                a = 0;
            }
        }
        a *= 1 / (1 - deadband);
        return a;
    }

    bool getAPressed() 
    {
        return joy->GetRawButtonPressed(1);
    }
};