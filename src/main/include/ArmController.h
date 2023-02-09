#pragma once
#include "Vector.h"
#include "cmath"
#include "math.h"

class ArmController{
    Vector *position;
    Vector *positionTarget;
    Vector *error;
    double speed;

    ArmController(double speed) {
        this->speed = speed;
        position = new Vector();
        positionTarget = new Vector();
        error = new Vector();
    }

    void update(Vector *positionTarget) {
        // moves the position toward the taget position
        this->error->set(positionTarget);
        this->error->subtractVector(position);
        if(error->getMagnitude() > 2*speed) {
            this->error->scaleFromUnitVector(speed);
        }
        else
        {
            this->error->scale(0.5);
        }
        position->addVector(error);
    }
};

