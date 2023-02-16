#pragma once
#include "Vector.h"
#include "cmath"
#include "math.h"

class ArmController{
    Vector position;
    Vector positionTarget;
    Vector error;
    double speed;
    const double extenderRPI = 18/25.4/5; // Revolutions of the motor needed for one inch of extension
    const double extenderIPR = 5*25.4/18; // inches of extension per revolution of the motor

    const double minExtension = 0;  // todo: set these values
    const double maxExtension = 0;
    const Vector startingPosition = Vector{};

    ArmController(double speed) {
        this->speed = speed;
    }

    void update(Vector positionTarget) {
        // moves the position toward the taget position
        error = positionTarget - position;
        if(abs(error) > 2*speed) {
            error *= speed/abs(error);
        }
        else
        {
            error *= 0.5;
        }
        position += error;
    }

    double getLeadscrewLength() {
        return sqrt(356-4*sqrt(7345)*cos(atan2(position.getX(), -position.getY())+atan(1.0/15.0)-atan(3.0/11.0)));
    }

    double getArmLength() {
        return abs(position);
    }
};

