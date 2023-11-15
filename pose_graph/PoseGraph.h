#ifndef POSEGRAPH_H_
#define POSEGRAPH_H_

#include <cmath>
#include <list>
#include <vector>
#include "Pose2D.h"
#include "GraphConstraint.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

/**
 * @brief Linear system struct used for saving linearized
 * parameters at each iteration in the graph optimization.
 */
struct LinearSystem
{
    /** @brief Dimension of the system for the linearized least-squares problem: 3*n (n: number of nodes). */
    uint dimension;

    /**
     * @brief State vector of the linear system, x is built by concatenating
     * all the robot global poses, (x_i, y_i, theta_i) for i = {0 to n}.
     */
    Eigen::VectorXd x;

    /**
     * @brief We want to solve the linear system (H * dx = -b).
     * H is the Hessian matrix, (information matrix of the Gaussian approximation).
     * b is the coefficient vector.
     * dx is the solution around the linearization point.
     */
    Eigen::VectorXd b;
    Eigen::VectorXd dx;
    Eigen::MatrixXd H;

    /** @brief Sparse matrix and solver for the linear system. */
    Eigen::SparseMatrix<double> HSparse;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;

    /** @brief Error tolerance and maximum number of iteraton for convergance. */
    double tolerance = 0.001;
    uint maxIter = 100;

    LinearSystem(uint dim):dimension(dim),x(dim), b(dim), dx(dim), H(dim, dim)
    {
        x = Eigen::VectorXd::Zero(dim);
        b = Eigen::VectorXd::Zero(dim);
        dx = Eigen::VectorXd::Zero(dim);
        H = Eigen::MatrixXd::Zero(dim, dim);
        HSparse.resize(dim, dim);
    }
};

/**
 * @brief PoseGraph Class for graph optimization, computing a Gaussian approximation of the
 * posterior over the robot's trajectory. A graph is consisting of a vector of graphNodes and a
 * linkedlist of graphConstraint.
 * The graph is constructed by adding robot poses either from odometry or loop closure detection,
 * by calling addOdomNode() or connectLoopClosingNodes().
 * The graph is optimized by calling optimizeGraph(fromNode, toNode), which optimizes the graph
 * iteratively between two given nodes.
 * @todo ---------------------------------------------------------------------------------------
 *## Tuning the inforamation matrix.
 *## Later, we want to assign an information matrix with each node.
 * ---------------------------------------------------------------------------------------------
 */
class PoseGraph
{

public:
    bool graphOptimized;
    GraphNode currentNode;
    GraphNode prevNode;
    GraphNode temp;
    GraphConstraint currentEdge;
    std::vector<GraphNode> graphNodes;
    std::list<GraphConstraint> graphConstraints;
     
private:
    static uint nodesCount;

    /** @brief Convergance condition for optimization part. */
    bool converged = false;

    /** @brief Correcting pose for the last node after each optimization. */
    Pose2D correctingPose;

    /**
     * @brief omega is the constant information Matrix for all nodes.
     * @todo Later, we want to assign an information matrix with each node.
     */
    Eigen::Matrix3d omega;

public:
    PoseGraph();

    /** @brief Constructor with adding the first pose as the prior node. */
    PoseGraph(const Pose2D& priorPose);

    ~PoseGraph();

    /** @brief Adds the first node to the graph if it is empty. */
    void addPriorNode(const Pose2D& priorPose);

    /** @brief Adds a node and a constraint to the graph from the odometry data. */
    void addOdomNode(const Pose2D &odomPose);

    /** @brief Adds a node and a constraint to the graph from the loop closing. */
    void connectLoopClosingNodes(uint nodeId1, uint nodeId2, const Pose2D &observation);

    /**
     * @brief Optimizes the graph between two nodes, by iteratively linearizing
     * the least-squares problem and solving the corresponding linear system.
     * This function will update the graph poses and constraints.
     */
    void optimizeGraph(const GraphNode &node1, const GraphNode &node2);

    Pose2D getCorrectingPose() const;
    static uint getNodesCount();

private:
    /** @brief Initializes the graph. */
    void init();

    /** @brief Updates the graph poses between two nodes, given by their Ids. */
    void updateGraphPoses(const Eigen::VectorXd &poseVector, uint nodeId1, uint nodeId2);

    /** @brief Updates the graph constraints. */
    void updateGraphConstraints();

    /** @todo
     * @brief Recovers the information matrix H of the computed mean of state x,
     * from the sparse matrix used in the linear solver.
     */
    void getInformationMatrix(Eigen::SparseMatrix<double> HSparse);
};

#endif
