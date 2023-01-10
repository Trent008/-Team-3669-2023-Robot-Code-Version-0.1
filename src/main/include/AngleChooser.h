#pragma once

/**
 * contains methods to find the most 
 * efficient way to turn from current given
 * angle to another given angle
 * */
class AngleChooser
{
    private:
        double option[4];
        double direction;
        double shortest;
    public:
        double getShortestAngle(double current, double setpoint) {
            option[0] = setpoint - current;
            if (option[0]>0) {option[1] = -360 + std::abs(option[0]);}
            else {option[1] = 360 + option[0];}
            if (option[0]>=0) {option[2] = -180 + std::abs(option[0]);}
            else {option[2] = option[0] + 180;}
            if (option[2]>=0) {option[3] = std::abs(option[2]-360);}
            else {option[3] = option[2] + 360;}

            // choose the lowest angle option
            shortest = option[0];
            for (int i = 1; i<4; i++)
            {
                if (std::abs(option[i]) < std::abs(shortest))
                {
                    shortest = option[i];
                }
            }
            if (shortest==option[2] || shortest==option[3]) {direction = -1;}
            else {direction = 1;}
            return shortest;
        }

        double getShortestDirection(double current, double setpoint) {
            option[0] = setpoint - current;
            if (option[0]>0) {option[1] = -360 + std::abs(option[0]);}
            else {option[1] = 360 + option[0];}

            // choose the lowest angle option
            shortest = option[0];
            if (std::abs(option[1]) < std::abs(shortest))
            {
                shortest = option[1];
            }
            return shortest;
        }


        double getDirection() {
            return direction;
        }
};