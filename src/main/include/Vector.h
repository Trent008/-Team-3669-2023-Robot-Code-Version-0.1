#pragma once
#include "cmath"
#include "math.h"

class Vector
{
private:
    double x, y, x_new, y_new;

public:
    // Default constructor
    Vector()
    {
        this->x = 0;
        this->y = 0;
    }

    Vector(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    // Method to set the coordinates as polar
    void setPolar(double magnitude, double angle)
    {
        this->x = magnitude * sin(angle * M_PI / 180);
        this->y = magnitude * cos(angle * M_PI / 180);
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
};

double abs(Vector &obj)
{
    return hypot(obj.getX(), obj.getY());
}

double angle(Vector &obj)
{
    return atan2(obj.getX(), obj.getY()) * 180 / M_PI;
}