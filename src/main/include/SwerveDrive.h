#pragma once
#include "FOC.h"
#include "SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"


/**
   * creates a swerve drive object that controls
   * an array of swerve drive modules
   * Parameters:
   *    FOC *motionController : for motion smoothing and F.O.C.
   *    SwerveModule *module[4] : array of 4 modules to control
   * */
class SwerveDrive
{
private:
  FOC* motionController;    // updates the robot velocity and rotation rate
  SwerveModule *module[4];  // array of 4 swerve modules
  Vector moduleVelocity[4];  // stores the velocity of all 4 modules
  Vector moduleTurnVector; // stores the turn-vector value for each module in turn
  Vector fieldwheelPositionChange; // stores the velocity of each wheel in encoders/100ms in turn
  Vector largestVector;    // fastest module velocity to be scaled to <= 1
  Vector averagePositionChange;  // average module velocity
  Vector fieldLocationEncoders;  // field location in encoders from the starting point
  Vector fieldLocation;    // field location in inches from the starting point 

public:
  SwerveDrive(FOC *motionController, SwerveModule *module[4])
  {
    this->motionController = motionController;
    for (int i = 0; i < 4; i++) {
      this->module[i] = module[i];
    }
  }

  /**
   * runs the swerve modules using the values from the motion controller
   **/
  void run() {
    for (int i = 0; i < 4; i++)     // compare all of the module velocities to find the largest
    {
      moduleTurnVector = module[i]->getTurnVector() * motionController->getRotationRate();
      moduleVelocity[i] = motionController->getRobotVelocity() + moduleTurnVector;
      if (abs(moduleVelocity[i]) > abs(largestVector))
      {
        largestVector = moduleVelocity[i]; 
      }
    }
    
    averagePositionChange = Vector{};    // reset the average to zero before averaging again

    for (int i = 0; i < 4; i++)  // loop through the module indexes again
    {
      if (abs(largestVector) > 1.0) {
        moduleVelocity[i] /= abs(largestVector);    // scale the vector sizes down to 1
      }
      
      module[i]->Set(moduleVelocity[i]);    // drive the modules

      fieldwheelPositionChange = module[i]->getwheelPositionChange();   // get the wheel velocity
      fieldwheelPositionChange.rotate(motionController->getRobotAngle());  // orient the wheel velocity in the robot's direction
      averagePositionChange += fieldwheelPositionChange;   // add the wheel velocity to the total sum
    }
    averagePositionChange /= 4; // find the average position change

    fieldLocationEncoders += averagePositionChange;  // adds the distance travelled this cycle to the total distance to find the position
  }

  Vector getPosition() {
    fieldLocation = fieldLocationEncoders;
    fieldLocation *= 1 / 8.41 / 2048 * M_PI * 3.9;  // converts the field location to inches
    return fieldLocation;
  }
};