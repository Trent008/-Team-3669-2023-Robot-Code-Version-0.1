#pragma once
#include "frc/Joystick.h"

class SpaceMouseEnt{
    private:
        frc::Joystick *joy;
    public:
        SpaceMouseEnt(frc::Joystick *joy) {
            this->joy = joy;
        }

        double getX() {
            return joy->GetRawAxis(0);
        }

        double getY() {
            return -joy->GetRawAxis(1);
        }

        double getZ() {
            return -joy->GetRawAxis(2)+0.05;
        }

        double getXR() {
            return -joy->GetRawAxis(3);
        }

        double GetYR() {
            return -joy->GetRawAxis(4);
        }

        double GetZR() {
            return joy->GetRawAxis(5);
        }
};