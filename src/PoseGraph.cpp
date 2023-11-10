#include "PoseGraph.h"
#include "util/StopWatch.h"
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
    graphNodes.append(currentNode);
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
    if (graphNodes.isEmpty())
    {
        currentNode.nodeId = nodesCount;
        currentNode.nodePose = priorPose;
        graphNodes.append(currentNode);
        nodesCount++;
    }
}

void PoseGraph::addOdomNode(const Pose2D &odomPose, const Vector<LinePair>& confirmedLinePairs)
{
    currentNode.nodeId = nodesCount;
    // Odometry poses are corrected by the error we get for the last pose, after optimization.
    currentNode.nodePose = odomPose;
    currentNode.initPose = currentNode.nodePose;

    // Storing the map lines, seen by the current pose.
    for (uint j = 0; j < confirmedLinePairs.size(); j++)
    {
        currentNode.seenMapLines << confirmedLinePairs[j].mapLine;
    }

    graphNodes.append(currentNode);
    currentNode.seenMapLines.clear();
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
    StopWatch sw;
    sw.start();

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
        ls.x(3*i+2) = pose.z;
    }

    uint numOfIter = 0;
    while(!converged)
    {
        /** @brief Linearizing the least-squares problem. */
        ls.H = Eigen::MatrixXd::Zero(dim, dim);
        ls.b = Eigen::VectorXd::Zero(dim);
        ListIterator<GraphConstraint> edgeIter = graphConstraints.begin();
        while (edgeIter.hasNext())
        {
            GraphConstraint& edge = edgeIter.next();
            // We only optimize the graph between two loop closing nodes.
            if (edge.i >= node1.nodeId)
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

        StopWatch sw;
        sw.start();

        ls.solver.compute(ls.HSparse);
        if(ls.solver.info()!=Eigen::Success)
        {
          qDebug() << "Decomposition failed at Iteration#: " << numOfIter;
          return;
        }

        ls.dx = ls.solver.solve(ls.b);
        if(ls.solver.info()!=Eigen::Success)
        {
          qDebug() << "Solving failed at Iteration#: " << numOfIter;
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
            double optTime = sw.elapsedTimeMs();
            qDebug() << "Optimization Successful! -- Total time:" << optTime << "(ms) -- No of Iterations:"
                     << numOfIter << "-- Delta X Norm:" << ls.dx.norm() << "\n";
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
    updateLineMap();
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
    ListIterator<GraphConstraint> edgeIter = graphConstraints.begin();
    while (edgeIter.hasNext())
    {
        GraphConstraint& edge = edgeIter.next();
        uint i = edge.i;
        uint j = edge.j;
        edge.updateConstraint(graphNodes[i], graphNodes[j]);
    }
}

void PoseGraph::updateLineMap()
{
    for (uint i = 1; i < graphNodes.size(); i++)
    {
        // For each node_i_ we iterate over all seen maplines by the node.
        for (uint j = 0; j < graphNodes[i].seenMapLines.size(); j++)
        {
            if (!graphNodes[i].seenMapLines[j]->updated)
            {
                // transforming the line_j_ by the new pose configuration.
                Pose2D trans = graphNodes[i].nodePose.diff(graphNodes[i].initPose);
                Line l = *graphNodes[i].seenMapLines[j];

                l.translate(trans.x, trans.y);
                Vec2& p1 = l.p1();
                Vec2& p2 = l.p2();
                diff(p1, graphNodes[i].nodePose);
                diff(p2, graphNodes[i].nodePose);
                l.rotate(trans.z);
                plus(p1, graphNodes[i].nodePose);
                plus(p2, graphNodes[i].nodePose);

                TrackedLine tl(l);
                tl.seenP1 = true;
                tl.seenP2 = true;

                // For each line_j_ in the mapline seen by the current node_i,
                // we check the next 70 nodes, if the line_j_ is seen by them,
                // we add the transformed of it by addLineObservation().
                for (uint k = i+1; k < i+70; k++)
                {
                    if (k < graphNodes.size() && graphNodes[k].seenMapLines.contains(graphNodes[i].seenMapLines[j]))
                    {
                        Pose2D trans = graphNodes[k].nodePose.diff(graphNodes[k].initPose);
                        TrackedLine* line = graphNodes[i].seenMapLines[j];
                        int index = graphNodes[k].seenMapLines.indexAt(line);
                        Line l = *graphNodes[k].seenMapLines[index];

                        l.translate(trans.x, trans.y);
                        Vec2& p1 = l.p1();
                        Vec2& p2 = l.p2();
                        diff(p1, graphNodes[k].nodePose);
                        diff(p2, graphNodes[k].nodePose);
                        l.rotate(trans.z);
                        plus(p1, graphNodes[k].nodePose);
                        plus(p2, graphNodes[k].nodePose);

                        tl.addLineObservation(l);
                        graphNodes[k].seenMapLines[index]->updated = true;
                    } // End_If
                } // End_For
                graphNodes[i].seenMapLines[j]->updated = true;
                graphNodes[i].seenMapLines[j]->p1() = tl.p1();
                graphNodes[i].seenMapLines[j]->p2() = tl.p2();
            } // End_If
        } // End_For
        graphNodes[i].initPose = graphNodes[i].nodePose;
    } // End_For

    // If we want to optimize the line map based on several loop closing for
    // the same lines, we have to put the line.updated flag to false again.
    for (uint i = 1; i < graphNodes.size(); i++)
    {
        for (uint j = 0; j < graphNodes[i].seenMapLines.size(); j++)
            graphNodes[i].seenMapLines[j]->updated = false;
    }
}

Pose2D PoseGraph::getCorrectingPose() const
{
    return correctingPose;
}

void PoseGraph::getInformationMatrix(Eigen::SparseMatrix<double> HSparse)
{}

void PoseGraph::diff(Vec2& vec, const Pose2D& p)
{
    double c = fcos(p.z);
    double s = fsin(p.z);
    double px = -c * p.x - s * p.y;
    double py = s * p.x - c * p.y;
    double x = c * vec.x + s * vec.y + px;
    double y = -s * vec.x + c * vec.y + py;
    vec.x = x;
    vec.y = y;
}

void PoseGraph::plus(Vec2& vec, const Pose2D& p)
{
    double c = cos(p.z);
    double s = sin(p.z);
    double x = c * vec.x - s * vec.y + p.x;
    double y = s * vec.x + c * vec.y + p.y;
    vec.x = x;
    vec.y = y;
}

QDebug operator<< (QDebug dbg, const PoseGraph &graph)
{
    dbg << "PoseGraph";
    return dbg;
}
