#pragma once
#include "Vector.h"
#include "AngleChooser.h"


// converts an 4 x 10 array giving coordinates, angles, and times into an autonomous setpoint array.
class AutoSetpointList
{
private:
    double mP = 30;
    double mA = 30;
    double p;
    double positionChangeTime;
    double angleChangeTime;
    double difference;
    double angleDifference;
    Vector *lastPosition;
    Vector *nextPosition;
    Vector *position;
    Vector *positionDifference;
    double angles[750];
    Vector *positions[750];
    AngleChooser angleChooser{};
    double changeTime;
    double positionRate;
    double angleRate;

public:
    AutoSetpointList(double setpoints[10][4]) {
        lastPosition = new Vector();
        nextPosition = new Vector();
        position = new Vector();
        positionDifference = new Vector();
        int t = 0;
        for (int n = 0; n < 750; n++) {
          positions[n] = new Vector();
        }
        for (int i = 1; i < 10; i++) {  // loops through the setpoint list
            lastPosition->set(setpoints[i-1][0], setpoints[i-1][1]); 
            nextPosition->set(setpoints[i][0], setpoints[i][1]);
            positionDifference->set(nextPosition);
            positionDifference->subtractVector(lastPosition);
            difference = positionDifference->getMagnitude();
            angleDifference = angleChooser.getShortestDirection(setpoints[i-1][2], setpoints[i][2]);
            // find the time it takes to reach the next setpoints
            positionChangeTime = difference/mP;
            angleChangeTime = std::abs(angleDifference)/mA;
            // find the slowest change time
            if (positionChangeTime >= angleChangeTime) {
                angleRate = angleChangeTime/positionChangeTime;
                positionRate = 1.0;
                changeTime = positionChangeTime;
            } else {
                positionRate = positionChangeTime/angleChangeTime;
                angleRate = 1.0;
                changeTime = angleChangeTime;
            }

            // break for the given time
            for (double j = 0; j < setpoints[i-1][3]; j += 0.02) {
                if (t < 750) {
                    positions[t]->set(lastPosition);
                    angles[t] = setpoints[i-1][2];
                    t++;
                }
            }
          
            // set setpoints
            for (double k = 0; k < changeTime; k += 0.02) {  // 
                if (t<750) {
                    position->setPolar(1, positionDifference->getAngle());
                    position->scale(getPathValue(k*positionRate, difference, mP));
                    position->addVector(lastPosition);
                    positions[t]->set(position);
                  
                    angles[t] = setpoints[i-1][2] + getPathValue(k*angleRate, angleDifference, mA);
                    t++;
                }
            }
        }
    }

    double getPathValue(double t, double d, double m) {
        if (d != 0) {
            p = m * t * std::abs(d)/d;
        } else {
            p = 0;
        }
        return p;
    }

    Vector *getPosition(int index) {
        return positions[index];
    }

    double getAngle(int index) {
        return angles[index];
    }
};