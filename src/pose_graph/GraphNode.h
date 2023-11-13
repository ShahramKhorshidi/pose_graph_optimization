#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#include "Pose2D.h"

/**
 * @brief GraphNode struct to be used inside the PoseGraph class.
 * A node is simply the 2d pose of the robot and its id.
 */

struct GraphNode
{
    uint nodeId;
    Pose2D nodePose;
    Pose2D initPose;
};

#endif // GRAPHNODE_H