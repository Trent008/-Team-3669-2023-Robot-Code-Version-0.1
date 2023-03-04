#pragma once
#include "Pose.h"

struct AutoSetpoint
{
    Pose pose = Pose{};
    Vector armPosition = {-9, 9.75};
    bool suctionCupState = false;
    double wristAngle = 0;
    double driveRate = 25;
    double rotationRate = 30;
};