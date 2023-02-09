#pragma once
#include "cmath"
#include "math.h"
class Vector {
    private:
        double x, y, x_new, y_new;

    public:
        // Default constructor
        Vector() {
            this->x = 0;
            this->y = 0;
        }

        Vector(double x, double y) {
            this->x = x;
            this->y = y;
        }

        // Method to set the coordinates as polar
        void setPolar(double magnitude, double angle) {
            this->x = magnitude * sin(angle*M_PI/180);
            this->y = magnitude * cos(angle*M_PI/180);
        }

        // Method to set the coordinates as cartesian
        void set(double x, double y) {
            this->x = x;
            this->y = y;
        }

        void set(Vector *other) {
            this->x = other->getX();
            this->y = other->getY();
        }

        // Method to rotate the vector by a given angle
        void rotate(double angle) {
            x_new = x * cos(-angle*M_PI/180) - y * sin(-angle*M_PI/180);
            y_new = x * sin(-angle*M_PI/180) + y * cos(-angle*M_PI/180);
            x = x_new;
            y = y_new;
        }

        // Method to add another vector to this vector
        void addVector(Vector *other) {
            this->x += other->getX();
            this->y += other->getY();
        }

        // Method to subtract another vector from this vector
        void subtractVector(Vector *other) {
            this->x -= other->getX();
            this->y -= other->getY();
        }

        // Method to scale the vector by a given value
        void scale(double scaleValue) {
            this->x *= scaleValue;
            this->y *= scaleValue;
        }

        // scale the vector from its unit vector
        void scaleFromUnitVector(double scaleValue) {
            scale(1.0/getMagnitude());
        }

        double getX() { return x; }

        double getY() { return y; }

        double getAngle() { return atan2(x, y)*180/M_PI; }

        double getMagnitude() { return sqrt(pow(x,2) + pow(y,2)); }
};
