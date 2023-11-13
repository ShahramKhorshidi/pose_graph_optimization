#include "PoseGraph.h"
#include "Pose2D.h"

int main()
{
    Pose2D priorPose = Pose2D(0, 0, 0);
    Pose2D prevPose = Pose2D(0, 0, 0);
    PoseGraph myPoseGraph = PoseGraph(priorPose);
    uint frameIdCount = 1;

    myPoseGraph.addOdomNode(snappedPose, confirmedLinePairs);

        // Loop closing nodes. nodId=133 and nodeId=749
        if (PoseGraph::getNodesCount() == 750)
        {
            qDebug() << "First loop closing.";
            uint nodeIdi = 133;
            uint nodeIdj = 749;

            // Relative Transformation between two poses from loop closing detection.
            Pose2D observation = Pose2D(0, 0 , -2.98);
            myPoseGraph.connectLoopClosingNodes(nodeIdi, nodeIdj, observation);

            // We only optimize the graph between two loop closing nodes.
            GraphNode& from = myPoseGraph.graphNodes[nodeIdi];
            GraphNode& to = myPoseGraph.graphNodes[nodeIdj];
            myPoseGraph.optimizeGraph(from, to);
        }


}    

