#pragma once
#include "cmath"
#include "math.h"

class Vector
{
private:
    double x, y, x_new, y_new;

public:
    /**
     * x = 0
     * y = 0
     **/
    Vector()
    {
        this->x = 0;
        this->y = 0;
    }

    /**
     * vector.x = x
     * vector.y = y
     **/
    Vector(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    // Method to rotate the vector by a given angle
    void rotate(double angle)
    {
        x_new = x * cos(-angle * M_PI / 180) - y * sin(-angle * M_PI / 180);
        y_new = x * sin(-angle * M_PI / 180) + y * cos(-angle * M_PI / 180);
        x = x_new;
        y = y_new;
    }

    void operator=(Vector const &obj)
    {
        x = obj.x;
        y = obj.y;
    }

    Vector operator+(Vector const &obj)
    {
        Vector res;
        res.x = x + obj.x;
        res.y = y + obj.y;
        return res;
    }

    Vector operator-(Vector const &obj)
    {
        Vector res;
        res.x = x - obj.x;
        res.y = y - obj.y;
        return res;
    }

    Vector operator*(double const &val)
    {
        Vector res;
        res.x = x * val;
        res.y = y * val;
        return res;
    }

    Vector operator/(double const &val)
    {
        Vector res;
        res.x = x / val;
        res.y = y / val;
        return res;
    }

    void operator*=(double const &val)
    {
        x *= val;
        y *= val;
    }

    void operator/=(double const &val)
    {
        x /= val;
        y /= val;
    }

    void operator+=(Vector const &obj)
    {
        x += obj.x;
        y += obj.y;
    }

    void operator-=(Vector const &obj)
    {
        x -= obj.x;
        y -= obj.y;
    }

    double getX() { return x; }

    double getY() { return y; }
    
    void setX(double x) {
        this->x = x;
    }

    void setY(double y) {
        this->y = y;
    }

};

double abs(Vector &obj)
{
    return hypot(obj.getX(), obj.getY());
}

double angle(Vector &obj)
{
    return atan2(obj.getX(), obj.getY()) * 180 / M_PI;
}

// creates a new vector using polar coordinates
Vector Polar(double magnitude, double angle)
{
    return Vector{magnitude * sin(angle * M_PI / 180), magnitude * cos(angle * M_PI / 180)};
}