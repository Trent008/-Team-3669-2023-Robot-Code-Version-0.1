#pragma once
#include "frc/Joystick.h"

class SpaceMouseEnt{
    private:
        frc::Joystick *joy;
    public:
        SpaceMouseEnt(frc::Joystick *joy) {
            this->joy = joy;
        }

        double x() {
            return joy->GetRawAxis(0);
        }

        double y() {
            return -joy->GetRawAxis(1);
        }

        double z() {
            return -joy->GetRawAxis(2)+0.05;
        }

        double xr() {
            return -joy->GetRawAxis(3);
        }

        double yr() {
            return -joy->GetRawAxis(4);
        }

        double zr() {
            return joy->GetRawAxis(5);
        }
};