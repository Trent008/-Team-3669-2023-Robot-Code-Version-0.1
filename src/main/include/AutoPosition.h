#pragma once;
#include "Vector.h"

class AutoPosition {
private:
    Vector position;
    Vector armPosition;
    bool suctionCupState;
    double angle;
    double wristAngle;
    double waitTime;
public:
    AutoPosition(Vector position = Vector{}, double angle = 0, Vector armPosition = Vector{}, bool suctionCupState = false, double wristAngle = 0, double waitTime = 0) {
        this->position = position;
        this->angle = angle;
        this->armPosition = armPosition;
        this->suctionCupState = suctionCupState;
        this->wristAngle = wristAngle;
        this->waitTime = waitTime;
    }

    Vector getPosition() {
        return position;
    }

    Vector getArmPosition() {
        return armPosition;
    }

    double getAngle() {
        return angle;
    }

    double getWristAngle() {
        return wristAngle;
    }

    double getWaitTime() {
        return waitTime;
    }

    bool getSuctionCupState() {
        return suctionCupState;
    }

    void operator=(AutoPosition const &obj)
    {
        position = obj.position;
        armPosition = obj.armPosition;
        angle = obj.angle;
        wristAngle = obj.wristAngle;
        suctionCupState = obj.suctionCupState;
        waitTime = obj.waitTime;
    }
};