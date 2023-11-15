#include <iostream>
#include <chrono>
#include "PoseGraph.h"
#include<eigen3/Eigen/SparseCholesky>

uint PoseGraph::nodesCount = 0;

uint PoseGraph::getNodesCount()
{
    return nodesCount;
}

PoseGraph::PoseGraph()
{
    init();
}

PoseGraph::PoseGraph(const Pose2D &priorPose)
{
    currentNode.nodeId = nodesCount;
    currentNode.nodePose = priorPose;
    currentNode.initPose = priorPose;
    graphNodes.push_back(currentNode);
    nodesCount++;
    init();
}

PoseGraph::~PoseGraph()
{
    nodesCount--;
}

void PoseGraph::init()
{
    /** @todo: Tuning the covariance matrix. */
    Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
    sigma(0, 0) = 0.15 * 0.15; // std_x = 0.15 m
    sigma(1, 1) = 0.15 * 0.15; // std_y = 0.15 m
    sigma(2, 2) = 0.2 * 0.2;   // std_theta = 0.2 rad
    omega = sigma.inverse();
    correctingPose = Pose2D(0, 0, 0);
    graphOptimized = false;
}

void PoseGraph::addPriorNode(const Pose2D& priorPose)
{
    if (graphNodes.empty())
    {
        currentNode.nodeId = nodesCount;
        currentNode.nodePose = priorPose;
        currentNode.initPose = priorPose;
        graphNodes.push_back(currentNode);
        nodesCount++;
    }
}

void PoseGraph::addOdomNode(const Pose2D &odomPose)
{
    currentNode.nodeId = nodesCount;
    // Odometry poses are corrected by the error we get for the last pose, after optimization.
    currentNode.nodePose = odomPose;
    currentNode.initPose = currentNode.nodePose;

    graphNodes.push_back(currentNode);
    nodesCount++;

    if (graphNodes.size() > 1)
    {
        uint i = graphNodes.size();
        currentEdge.addOdomConstraint(graphNodes[i-2], graphNodes[i-1]);
        graphConstraints.push_back(currentEdge);
    }
}

void PoseGraph::connectLoopClosingNodes(uint nodeId1, uint nodeId2, const Pose2D &observation)
{
    GraphNode& nodei = graphNodes[nodeId1];
    GraphNode& nodej = graphNodes[nodeId2];
    currentEdge.addLoopClosingConstraint(nodei, nodej, observation);
    graphConstraints.push_back(currentEdge);
}

void PoseGraph::optimizeGraph(const GraphNode &node1, const GraphNode &node2)
{
    auto opt_start = std::chrono::high_resolution_clock::now();
    // Storing the last pose of the robot before the optimization.
    Pose2D beforeOptimization = graphNodes[nodesCount-1].nodePose;
    
    // Creating the linear sysyem.
    uint NoOfNodes = node2.nodeId - node1.nodeId + 1;
    uint dim = 3 * NoOfNodes;
    LinearSystem ls(dim);

    // Initial guess for ls.x, we set it as current graph nodes pose.
    for (uint i = 0; i < NoOfNodes; i++)
    {
        Pose2D pose = graphNodes[node1.nodeId+i].nodePose;
        ls.x(3*i) = pose.x;
        ls.x(3*i+1) = pose.y;
        ls.x(3*i+2) = pose.theta;
    }

    uint numOfIter = 0;
    while(!converged)
    {
        /** @brief Linearizing the least-squares problem. */
        ls.H = Eigen::MatrixXd::Zero(dim, dim);
        ls.b = Eigen::VectorXd::Zero(dim);
        for (auto& edge : graphConstraints)
        {
            // We only optimize the graph between two loop closing nodes.
            if (edge.i >= node1.nodeId && edge.i <= node2.nodeId)
            {
                edge.linearizeConstraint();
                uint i = edge.i-node1.nodeId;
                uint j = edge.j-node1.nodeId;
                ls.H.block<3, 3>(3*i, 3*i) += edge.getAij().transpose() * omega * edge.getAij();
                ls.H.block<3, 3>(3*i, 3*j) += edge.getAij().transpose() * omega * edge.getBij();
                ls.H.block<3, 3>(3*j, 3*i) += edge.getBij().transpose() * omega * edge.getAij();
                ls.H.block<3, 3>(3*j, 3*j) += edge.getBij().transpose() * omega * edge.getBij();

                ls.b.block<3, 1>(3*i, 0) += edge.getAij().transpose() * omega * edge.getErrorVector();
                ls.b.block<3, 1>(3*j, 0) += edge.getBij().transpose() * omega * edge.getErrorVector();
            }
        }
        // Keep the first node fixed.
        ls.H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity();

        /** @brief Sparse solver. */
        ls.HSparse = ls.H.sparseView();

        ls.solver.compute(ls.HSparse);
        if(ls.solver.info()!=Eigen::Success)
        {
          std::cout << "Decomposition failed at Iteration#: " << numOfIter << std::endl;
          return;
        }

        ls.dx = ls.solver.solve(ls.b);
        if(ls.solver.info()!=Eigen::Success)
        {
          std::cout << "Solving failed at Iteration#: " << numOfIter << std::endl;
          return;
        }
        // Update the parameters.
        ls.x += ls.dx;

        // Update the grpah poses and constraints, at each iteration.
        updateGraphPoses(ls.x, node1.nodeId, node2.nodeId);
        updateGraphConstraints();
        numOfIter++;

        if(ls.dx.norm() < ls.tolerance || numOfIter > ls.maxIter)
        {
            auto opt_stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(opt_stop - opt_start);
            std::cout << "\n==== Optimization Successful! ==== " << 
                         "\n --- Total time=" << duration.count() << "(ms)" <<
                         "\n --- No of Iterations=" << numOfIter << 
                         "\n --- Delta_X Norm=" << ls.dx.norm() << std::endl;
            // Storing the correcting pose for the last pose after optimization.
            Pose2D& afterOptimization = graphNodes[nodesCount-1].nodePose;
            correctingPose = afterOptimization.diff(beforeOptimization);
            converged = true;
        }
    }
    /** @todo Recover the information matrix H from the HSparse. */
    getInformationMatrix(ls.HSparse);

    // Setting the condition false, for the next optimization loop.
    converged = false;

    // Updating the line map with the new pose configuration.
    graphOptimized = true;
}

void PoseGraph::updateGraphPoses(const Eigen::VectorXd &poseVector, uint nodeId1, uint nodeId2)
{
    uint NoOfNodes = nodeId2 - nodeId1 + 1;
    for (uint i = 0; i < NoOfNodes; i++)
    {
        double x = poseVector(3*i);
        double y = poseVector(3*i+1);
        double z = poseVector(3*i+2);
        Pose2D newPose = Pose2D(x, y, z);

        graphNodes[nodeId1+i].nodePose = newPose;
    }
}

void PoseGraph::updateGraphConstraints()
{
    for (auto& edge : graphConstraints)
    {
        uint i = edge.i;
        uint j = edge.j;
        edge.updateConstraint(graphNodes[i], graphNodes[j]);
    }
}

Pose2D PoseGraph::getCorrectingPose() const
{
    return correctingPose;
}

void PoseGraph::getInformationMatrix(Eigen::SparseMatrix<double> HSparse)
{
    
}