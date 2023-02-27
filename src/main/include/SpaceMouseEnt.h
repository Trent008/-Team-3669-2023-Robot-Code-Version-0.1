#pragma once
#include "frc/Joystick.h"
#include "math.h"
#include "cmath"

class SpaceMouseEnt{
    private:
        frc::Joystick *joy;
        double a;
    public:
        SpaceMouseEnt(frc::Joystick *joy) {
            this->joy = joy;
        }

        double getX() {
            a = 0.7 * joy->GetRawAxis(0);
            return std::signbit(a) ? -pow(a, 2): pow(a, 2);
        }

        double getY() {
            a = 0.7 * -joy->GetRawAxis(1);
            return std::signbit(a) ? -pow(a, 2): pow(a, 2);
        }

        double getZ() {
            return -joy->GetRawAxis(2)+0.05;
        }

        double getXR() {
            return -joy->GetRawAxis(3);
        }

        double getYR() {
            return -joy->GetRawAxis(4);
        }

        double getZR() {
            a = 0.7 * joy->GetRawAxis(5);
            return std::signbit(a) ? -pow(a, 2): pow(a, 2);
        }
};