//
// Created by kadupitiya on 10/12/18.
// This is 3d vector class (cartesian coordinate system)
//

#ifndef PROJECT_VECTOR2D_H
#define PROJECT_VECTOR2D_H

#include <iostream>
#include<cmath>


class VECTOR2D
{

public:
    double x, y;								// component along each axis (cartesian)

    // make a 3d vector
    VECTOR2D(double initial_x = 0.0, double initial_y = 0.0)
    {
        x = initial_x;
        y = initial_y;
    }

    double getMagnitude()								// magnitude of the vector
    {
        return sqrt(x*x + y*y);
    }
    double getMagnitudeSquared()							// magnitude squared of the vector
    {
        return (x*x + y*y);
    }

    // overloading -, *, ^, +, ==, +=
    VECTOR2D operator-(const VECTOR2D& vec)						// subtract two vectors
    {
        return VECTOR2D(x - vec.x, y - vec.y);
    }
    double operator*(const VECTOR2D& vec)						// dot product of two vectors
    {
        return x*vec.x + y*vec.y;
    }
    VECTOR2D operator^(double scalar)						// product of a vector and a scalar
    {
        return VECTOR2D(x*scalar, y*scalar);
    }
    VECTOR2D operator+(const VECTOR2D& vec)						// add two vectors
    {
        return VECTOR2D(x + vec.x, y + vec.y);
    }
    bool operator==(const VECTOR2D& vec)						// compare two vectors
    {
        if (x==vec.x && y==vec.y)
            return true;
        return false;
    }
    void operator+=(const VECTOR2D& vec)
    {
        x += vec.x;
        y += vec.y;
    }
};

#endif //PROJECT_VECTOR2D_H
