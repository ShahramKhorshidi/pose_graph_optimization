#ifndef POSE2D_H
#define POSE2D_H

class Pose2D
{
public:

    double x,y,theta;

    Pose2D();
    Pose2D(double x, double y, double theta);

    Pose2D operator-(const Pose2D &o) const;
    Pose2D operator+(const Pose2D &o) const;
    void operator-=(const Pose2D &o);
    void operator+=(const Pose2D &o);
    
    double ffpicut(double theta) const;
    Pose2D diff(const Pose2D& p1) const;
};

#endif