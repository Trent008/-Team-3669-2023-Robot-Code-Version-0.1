#pragma once
#include "AngleChooser.h"
#include "Vector.h"

// converts an 4 x 10 array giving coordinates, angles, and times into an
// autonomous setpoint array.
class AutoSetpointList {
private:
  double mP = 30;
  double mA = 30;

public:
  double p;
  double positionChangeTime;
  double angleChangeTime;
  double difference;
  double angleDifference;
  Vector lastPosition;
  Vector nextPosition;
  Vector position;
  Vector positionDifference;
  double angles[750];
  Vector positions[750];
  AngleChooser angleChooser{};
  double changeTime;
  double positionRate;
  double angleRate;
  AutoSetpointList(double setpoints[10][4]) {
    int t = 0;
    for (int i = 1; i < 10; i++) { // loops through the setpoint list
      lastPosition = Vector{setpoints[i - 1][0], setpoints[i - 1][1]};
      nextPosition = Vector{setpoints[i][0], setpoints[i][1]};
      positionDifference = nextPosition - lastPosition;
      difference = abs(positionDifference);
      angleDifference = angleChooser.getShortestDirection(setpoints[i - 1][2],
                                                          setpoints[i][2]);
      // find the time it takes to reach the next setpoints
      positionChangeTime = difference / mP;
      angleChangeTime = std::abs(angleDifference) / mA;
      // find the slowest change time
      if (positionChangeTime >= angleChangeTime) {
        angleRate = angleChangeTime / positionChangeTime;
        positionRate = 1.0;
        changeTime = positionChangeTime;
      } else {
        positionRate = positionChangeTime / angleChangeTime;
        angleRate = 1.0;
        changeTime = angleChangeTime;
      }

      // break for the given time
      for (double j = 0; j < setpoints[i - 1][3]; j += 0.02) {
        if (t < 750) {
          positions[t] = lastPosition;
          angles[t] = setpoints[i - 1][2];
          t++;
        }
      }

      // set setpoints
      for (double k = 0; k < changeTime; k += 0.02) { //
        if (t < 750) {
          position.setPolar(1, angle(positionDifference));
          position *= getPathValue(k * positionRate, difference, mP);
          position += lastPosition;
          positions[t] = position;

          angles[t] = setpoints[i - 1][2] +
                      getPathValue(k * angleRate, angleDifference, mA);
          t++;
        }
      }
    }
  }

  double getPathValue(double t, double d, double m) {
    if (d != 0) {
      p = m * t * std::abs(d) / d;
    } else {
      p = 0;
    }
    return p;
  }

  Vector getPosition(int index) { return positions[index]; }

  double getAngle(int index) { return angles[index]; }
};
