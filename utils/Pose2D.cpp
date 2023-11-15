#include <cmath>
#include "Pose2D.h"
const double EPSILON = 1.0E-6;
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

// Maps this pose from world coordinates into the coordinate frame of the given Pose2D o.
void Pose2D::operator-=(const Pose2D &o)
{
    x -= o.x;
    y -= o.y;
    theta -= o.theta;
    theta = ffpicut(theta);

    if (o.theta < EPSILON && o.theta > -EPSILON)
        return;

    double c = std::cos(-o.theta);
    double s = std::sin(-o.theta);
    double x_ = x;
    x = x_*c + y*-s;
    y = x_*s + y*c;

    return;
}

// Maps this pose from the coordinate frame of the given Pose2D o to world coordinates.
void Pose2D::operator+=(const Pose2D &o)
{
    if (o.theta > EPSILON || o.theta < -EPSILON)
    {
        double c = std::cos(o.theta);
        double s = std::sin(o.theta);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    x += o.x;
    y += o.y;
    theta += o.theta;
    theta = ffpicut(theta);
}

//Returns the (p2-p1) in the world coordinate.
Pose2D Pose2D::diff(const Pose2D& p1) const
{
    Pose2D diff;
    diff.x = this->x - p1.x;
    diff.y = this->y - p1.y;
    diff.theta = this->theta - p1.theta;
    return diff;
}

double Pose2D::ffpicut(double theta) const
{
    theta = std::fmod((theta + M_PI), (2 * M_PI));
    if (theta < 0)
        theta += (2*M_PI);
    theta -= M_PI;
    return theta;
}