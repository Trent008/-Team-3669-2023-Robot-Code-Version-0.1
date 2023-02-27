#pragma once
#include "AngleChooser.h"
#include "AutoPosition.h"
#include "Vector.h"

// converts an 4 x 10 array giving coordinates, angles, and times into an
// autonomous setpoint array.
class AutoSetpointList
{
private:
  double mP = 10;
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
  AutoPosition positions[750];
  AngleChooser angleChooser{};
  double changeTime;
  double positionRate;
  double angleRate;
  AutoSetpointList(AutoPosition setpoints[10])
  {
    int t = 0;
    for (int i = 1; i < 10; i++)
    { // loops through the setpoint list
      lastPosition = setpoints[i - 1].getPosition();
      nextPosition = setpoints[i].getPosition();
      positionDifference = nextPosition - lastPosition;
      difference = abs(positionDifference);
      angleDifference = angleChooser.getShortestDirection(setpoints[i - 1].getAngle(), setpoints[i].getAngle());
      // find the time it takes to reach the next setpoints
      positionChangeTime = difference / mP;
      angleChangeTime = std::abs(angleDifference) / mA;
      // find the slowest change time
      if (positionChangeTime >= angleChangeTime)
      {
        angleRate = angleChangeTime / positionChangeTime;
        positionRate = 1.0;
        changeTime = positionChangeTime;
      }
      else
      {
        positionRate = positionChangeTime / angleChangeTime;
        angleRate = 1.0;
        changeTime = angleChangeTime;
      }

      // break for the given time
      for (double j = 0; j < setpoints[i - 1].getWaitTime(); j += 0.02)
      {
        if (t < 750)
        {
          positions[t] = setpoints[i-1];
          t++;
        }
      }

      // set setpoints
      for (double k = 0; k < changeTime; k += 0.02)
      { //
        if (t < 750)
        {
          position = Polar(1, angle(positionDifference));
          position *= getPathValue(k * positionRate, difference, mP);
          position += lastPosition;
          positions[t] = AutoPosition{position, setpoints[i - 1].getAngle() + getPathValue(k * angleRate, angleDifference, mA), setpoints[i].getArmPosition(), setpoints[i].getSuctionCupState(), setpoints[i].getWristAngle()};
          t++;
        }
      }
    }
  }

  double getPathValue(double t, double d, double m)
  {
    if (d != 0)
    {
      p = m * t * std::abs(d) / d;
    }
    else
    {
      p = 0;
    }
    return p;
  }

  AutoPosition getPose(int index) {
    return positions[index];
  }
};
