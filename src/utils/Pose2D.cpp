#include "Pose2D.h"

// Constructs a default Pose2D with (x,y,z) = (0,0,0).
Pose2D::Pose2D()
{
    x = 0;
    y = 0;
    theta = 0;
}

// Constructs a Pose2D with (x,y,z) = (xo,yo,zo).
Pose2D::Pose2D(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

// Maps this pose into the coordinate frame of the given Pose2D o.
Pose2D Pose2D::operator-(const Pose2D &o) const
{
    Pose2D c = *this;
    c -= o;
    return c;
}

// Maps this pose from the coordinate frame of the given Pose2D o to world coordinates.
Pose2D Pose2D::operator+(const Pose2D &o) const
{
    Pose2D c = *this;
    c += o;
    return c;
}