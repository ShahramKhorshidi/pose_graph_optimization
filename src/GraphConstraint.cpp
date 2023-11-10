#include "GraphConstraint.h"
#include "globals.h"
#include <cmath>

Eigen::Matrix3d GraphConstraint::getAij() const
{
    return Aij;
}

Eigen::Matrix3d GraphConstraint::getBij() const
{
    return Bij;
}

Eigen::Vector3d GraphConstraint::getErrorVector() const
{
    return errorVector;
}

void GraphConstraint::addOdomConstraint(const GraphNode &node1, const GraphNode &node2)
{
    // We add the constraint from node1 to node2, so i<j.
    i = node1.nodeId;
    j = node2.nodeId;
    posei = node1.nodePose;
    posej = node2.nodePose;

    // Both observation and measurement are expressed in frame_{i}.
    observation = posej - posei;
    observation.z = normalizeTheta(observation.z);

    // We set the measuremet equal to observation, as the best guess we have
    // for the initial state to start the optimization later.
    measurement = observation;
}

void GraphConstraint::addLoopClosingConstraint(
        const GraphNode &node1,
        const GraphNode &node2,
        const Pose2D &observation)
{
    // We add the constraint from node1 to node2, so i<j.
    i = node1.nodeId;
    j = node2.nodeId;
    posei = node1.nodePose;
    posej = node2.nodePose;
    measurement = posej - posei;
    measurement.z = normalizeTheta(measurement.z);
    this->observation = observation;
    this->observation.z = normalizeTheta(this->observation.z);
    qDebug() << "Pose i=" << i << "---" << posei;
    qDebug() << "Pose j=" << j << "---"  << posej << "\n";
}
void GraphConstraint::linearizeConstraint()
{
    constructErrorJacobianA();
    constructErrorJacobianB();
    computeErrorVector();
}

void GraphConstraint::constructErrorJacobianA()
{
    Aij = Eigen::Matrix3d::Zero();
    double theta = posei.z;
    Aij.block<2, 2>(0, 0) = -getRotationMatrix(theta).transpose();
    Eigen::Vector2d t_ji;
    t_ji(0) = posej.x - posei.x;
    t_ji(1) = posej.y - posei.y;
    Aij.block<2, 1>(0, 2) = getRotationMatrixDerivative(theta).transpose() * t_ji;
    Aij(2, 2) = -1;
}

void GraphConstraint::constructErrorJacobianB()
{
    Bij = Eigen::Matrix3d::Zero();
    double theta = posei.z;
    Bij.block<2, 2>(0, 0) = getRotationMatrix(theta).transpose();
    Bij(2, 2) = 1;
}

void GraphConstraint::computeErrorVector()
{
    Pose2D error = Pose2D(0, 0, 0);
    error = observation.diff(measurement);
    error.z = normalizeTheta(error.z);

    errorVector(0) = error.x;
    errorVector(1) = error.y;
    errorVector(2) = error.z;
}

void GraphConstraint::updateConstraint(const GraphNode &node1, const GraphNode &node2)
{
    posei = node1.nodePose;
    posej = node2.nodePose;
    measurement = posej - posei;
    measurement.z = normalizeTheta(measurement.z);
}

double GraphConstraint::normalizeTheta(double theta) const
{
    theta = std::fmod((theta + PI), (2 * PI));
    if (theta < 0)
        theta += (2*PI);
    theta -= PI;
    return theta;
}

Eigen::Matrix2d GraphConstraint::getRotationMatrix(double theta) const
{
    Eigen::Matrix2d rotMatrix;
    rotMatrix(0, 0) = fcos(theta);
    rotMatrix(0, 1) = -fsin(theta);
    rotMatrix(1, 0) = fsin(theta);
    rotMatrix(1, 1) = fcos(theta);
    return rotMatrix;
}

Eigen::Matrix2d GraphConstraint::getRotationMatrixDerivative(double theta) const
{
    Eigen::Matrix2d rotMatrix;
    rotMatrix(0, 0) = -fsin(theta);
    rotMatrix(0, 1) = -fcos(theta);
    rotMatrix(1, 0) = fcos(theta);
    rotMatrix(1, 1) = -fsin(theta);
    return rotMatrix;
}

QDebug operator<< (QDebug dbg, const GraphConstraint &constraint)
{
    dbg << "PoseGraph";
    return dbg;
}
