#pragma once
#include "frc/Joystick.h"

class XBOXController{
    private:
        frc::Joystick *joy;
    public:
        XBOXController(frc::Joystick *joy) {
            this->joy = joy;
        }

        double x() {
            return joy->GetRawAxis(0);
        }

        double y() {
            return -joy->GetRawAxis(1);
        }

        double z() {
            return joy->GetRawAxis(4);
        }

        double getX() {
            return (std::abs(x()) > 0.1 ? x() : 0);
        }

        double getY() {
            return (std::abs(y()) > 0.1 ? y() : 0);
        }

        double getZ() {
            return (std::abs(z()) > 0.1 ? z() : 0);
        }
};