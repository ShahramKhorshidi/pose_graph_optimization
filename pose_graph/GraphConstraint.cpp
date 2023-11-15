#include <cmath>
#include "GraphConstraint.h"

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
    observation.theta = normalizeTheta(observation.theta);

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
    measurement.theta = normalizeTheta(measurement.theta);
    this->observation = observation;
    this->observation.theta = normalizeTheta(this->observation.theta);
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
    double theta = posei.theta;
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
    double theta = posei.theta;
    Bij.block<2, 2>(0, 0) = getRotationMatrix(theta).transpose();
    Bij(2, 2) = 1;
}

void GraphConstraint::computeErrorVector()
{
    Pose2D error = Pose2D(0, 0, 0);
    error = observation.diff(measurement);
    error.theta = normalizeTheta(error.theta);

    errorVector(0) = error.x;
    errorVector(1) = error.y;
    errorVector(2) = error.theta;
}

void GraphConstraint::updateConstraint(const GraphNode &node1, const GraphNode &node2)
{
    posei = node1.nodePose;
    posej = node2.nodePose;
    measurement = posej - posei;
    measurement.theta = normalizeTheta(measurement.theta);
}

double GraphConstraint::normalizeTheta(double theta) const
{
    theta = std::fmod((theta + M_PI), (2 * M_PI));
    if (theta < 0)
        theta += (2*M_PI);
    theta -= M_PI;
    return theta;
}

Eigen::Matrix2d GraphConstraint::getRotationMatrix(double theta) const
{
    Eigen::Matrix2d rotMatrix;
    rotMatrix(0, 0) = std::cos(theta);
    rotMatrix(0, 1) = -std::sin(theta);
    rotMatrix(1, 0) = std::sin(theta);
    rotMatrix(1, 1) = std::cos(theta);
    return rotMatrix;
}

Eigen::Matrix2d GraphConstraint::getRotationMatrixDerivative(double theta) const
{
    Eigen::Matrix2d rotMatrix;
    rotMatrix(0, 0) = -std::sin(theta);
    rotMatrix(0, 1) = -std::cos(theta);
    rotMatrix(1, 0) = std::cos(theta);
    rotMatrix(1, 1) = -std::sin(theta);
    return rotMatrix;
}