#ifndef GRAPHCONSTRAINT_H
#define GRAPHCONSTRAINT_H

#include "Pose2D.h"
#include "GraphNode.h"
#include <eigen3/Eigen/Dense>

/**
 * @brief GraphConstraint class to be used inside the PoseGraph class. A constraint is added from two
 * nodes, by calling addOdomConstraint() or addLoopClosingConstraint(), Later for the graph optimazation,
 * we compute the jacobians and error vector, over the constraint by calling linearizeConstraint(),
 */

class GraphConstraint
{
public:
    /** @brief Id's of connected nodes by this constraint. */
    uint i, j;
    Eigen::Matrix3d omega;

private:
    /** @brief Poses of node i and node j. */
    Pose2D posei;
    Pose2D posej;

    Pose2D measurement;
    Pose2D observation;

    /** @brief Jacobian matrices and error vector for Linearization. */
    Eigen::Matrix3d Aij = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Bij = Eigen::Matrix3d::Zero();
    Eigen::Vector3d errorVector = Eigen::Vector3d::Zero();

public:
    GraphConstraint(){}
    ~GraphConstraint(){}

    /** @brief Getter for jacobians and error vector. */
    Eigen::Matrix3d getAij() const;
    Eigen::Matrix3d getBij() const;
    Eigen::Vector3d getErrorVector() const;

    /** @brief Adds the corresponding constraint from odometry between two consecutive nodes. */
    void addOdomConstraint(const GraphNode &node1, const GraphNode &node2);

    /** @brief Adds the corresponding constraint from loop closing between two nodes. */
    void addLoopClosingConstraint(
            const GraphNode &node1,
            const GraphNode &node2,
            const Pose2D &observation);

    /** @brief Computes the jacobians and error vector over the constraint between two nodes. */
    void linearizeConstraint();

    /** @brief Updates the graph constraint (posei, posej and measurement), from new node configuration. */
    void updateConstraint(const GraphNode &node1, const GraphNode &node2);

private:
    /** @brief Computes the jacobian of error function w.r.t c{i}, partial_e{ij}/partial_c{i}. */
    void constructErrorJacobianA();

    /** @brief Computes the jacobian of error function w.r.t c{j}, partial_e{ij}/partial_c{j}. */
    void constructErrorJacobianB();

    /** @brief Computes the error vector e{ij}, from observation and mesurement. */
    void computeErrorVector();

    /** @brief Returns the normalized angle, between [-PI, +PI) */
    double normalizeTheta(double theta) const;

    /** @brief Returns the rotation matrix SO(2).*/
    Eigen::Matrix2d getRotationMatrix(double theta) const;

    /** @brief Returns the derivative of rotation matrix SO(2).*/
    Eigen::Matrix2d getRotationMatrixDerivative(double theta) const;
};

#endif // GRAPHCONSTRAINT_H

