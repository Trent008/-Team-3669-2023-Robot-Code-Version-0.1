#pragma once
#include "frc/Joystick.h"
#include "Vector.h"
#include <frc/smartdashboard/SmartDashboard.h>

class SpaceMouse
{
private:
    frc::Joystick *joy;
    double speed;
    double change[6];
    double lastValue[6], value[6];
    int cycles = 0;

public:
    // SpaceMouse as joystick
    SpaceMouse(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    /**
     * resets the spaceMouse last position variable
     * to the current output
     * */
    void initialize(double speed) {
        this->speed = speed;
        for (int i = 0; i < 6; i++) {
            lastValue[i] = joy->GetRawAxis(i);
        }
    }

    void update() {
        if (cycles == 3) {
            for (int i = 0; i < 6; i++) {
                value[i] = joy->GetRawAxis(i);
                change[i] = value[i] - lastValue[i];
                if (change[i] > .9) {
                    change[i] -= 2.0;
                }
                if (change[i] < -.9) {
                    change[i] += 2.0;
                }
                lastValue[i] = value[i];
                change[i] *= speed/3.0;
            }
            cycles = 0;
        }
        cycles++;
    }

    double getAxis(int number) {
        return change[number];
    }
};