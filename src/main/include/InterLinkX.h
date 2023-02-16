#pragma once
#include "frc/Joystick.h"

class InterLinkX{
    private:
        frc::Joystick *joy;
    public:
        InterLinkX(frc::Joystick *joy) {
            this->joy = joy;
        }

        double x() {
            return joy->GetRawAxis(0);
        }

        double y() {
            return -joy->GetRawAxis(1);
        }

        double z() {
            return joy->GetRawAxis(5)+0.05;
        }

        double getX() {
            return (std::abs(x()) > 0.05 ? x() : 0);
        }

        double getY() {
            return (std::abs(y()) > 0.05 ? y() : 0);
        }

        double getZ() {
            return (std::abs(z()) > 0.05 ? z() : 0);
        }
};