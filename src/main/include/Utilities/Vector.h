#pragma once
#include <math.h>

class Vector
{
    private:
        double magnitude, angle;
    public:
        /**
         * creates a polar or cartesian vector
         * 
         * if isPolar = true, the vector is polar
         * with a = angle and b = magnitude
         * 
         * if isPolar = false, the vector is cartesian
         * with a = x-component and b = y-component
         **/
        Vector(double a, double b, bool isPolar) {
            if (isPolar) {
                angle = a;
                magnitude = b;
            } else {
                if (b>0) {angle = atan(a/b) * 180/M_PI;}
                else if (b==0 and a>0) {angle = 90;}
                else if (b==0 and a<0) {angle = -90;}
                else if (b==0 and a==0) {angle = 0;}
                else {angle = atan(a/b) * 180/M_PI+180;}
                if (angle>180) {angle -= 360;}
                magnitude = sqrt(pow(a, 2) + pow(b, 2));
            }
        }

        Vector(Vector *vector) {
            this->magnitude = vector->getMagnitude();
            this->angle = vector->getAngle();
        }

        /**
         * sets the vector values using polar or cartesian
         * coordinates
         * 
         * if isPolar = true, set vector to
         * a = angle and b = magnitude
         * 
         * if isPolar = false, set vector to
         * a = x-component and b = y-component
         **/
        void set(double a, double b, bool isPolar) {
            if (isPolar) {
                angle = a;
                magnitude = b;
            } else {
                if (b>0) {angle = atan(a/b) * 180/M_PI;}
                else if (b==0 and a>0) {angle = 90;}
                else if (b==0 and a<0) {angle = -90;}
                else if (b==0 and a==0) {angle = 0;}
                else {angle = atan(a/b) * 180/M_PI+180;}
                if (angle>180) {angle -= 360;}
                magnitude = sqrt(pow(a, 2) + pow(b, 2));
            }
        }

        void set(Vector *vector) {
            set(vector->getAngle(), vector->getMagnitude(), true);
        }

        double getAngle()
        {
            return angle;
        }

        double getMagnitude()
        {
            return magnitude;
        }

        double getXComponent() {
            return magnitude * sin(angle * M_PI / 180);
        }

        double getYComponent() {
            return magnitude * cos(angle * M_PI / 180);
        }

        void rotate (double angle) {
            this-> angle += angle;
            while (this->angle > 180) {
                this->angle -= 360;
            }
            while (this->angle <= -180) {
                this->angle +=360;
            }
            if (this->angle > 180) {this->angle -= 360;}
        }

        void scale(double scaleValue) {
            magnitude *= scaleValue;
        }

        //returns a vector that is the sum of this vector and the given vector
        void addVector(Vector *vector) {
            set(this->getXComponent() + vector->getXComponent(), this->getYComponent() + vector->getYComponent(), false);
        }

        void subtractVector(Vector *vector) {
            set(this->getXComponent() - vector->getXComponent(), this->getYComponent() - vector->getYComponent(), false);
        }
};