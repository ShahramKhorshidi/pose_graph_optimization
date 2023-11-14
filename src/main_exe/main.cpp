#include <QtMath> 
#include <QTime>
#include <QTimer>
#include <QElapsedTimer>
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "Pose2D.h"
#include "PoseGraph.h"

struct Trajectory {
    float id, x, y, angle;
};

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    // Create a QGraphicsScene
    QGraphicsScene scene;
    scene.setSceneRect(-400, -1400, 1000, 1600); // Set scene size

    // Read trajectory data from a file
    std::ifstream inputFile("/home/khorshidi/git/pose_graph_optimization/src/main_exe/robot_trajectory.log");
    if (!inputFile.is_open()) {
        std::cerr << "Error opening input file" << std::endl;
        return -1;
    }

    std::vector<Trajectory> trajectory;
    float id, x, y, angle;
    while (inputFile >> id >> x >> y >> angle) {
        trajectory.push_back({id, x, y, angle});
    }

    // Load an image for the robot
    QPixmap robotImage("/home/khorshidi/git/pose_graph_optimization/src/main_exe/robot_image1.png");  // Replace with your image file path

    // Create a QElapsedTimer to measure time
    QElapsedTimer timer;
    timer.start();

    // Create a QGraphicsView to visualize the scene
    QGraphicsView view(&scene);
    view.scale(-1, 1);
    view.show();

    // Instantiating a pose graph object
    PoseGraph poseGraph = PoseGraph();

    // Draw trajectory on the scene
    for (const Trajectory& robotTraj : trajectory) {
        // Adding the first node to the pose graph
        if (robotTraj.id == 0){
            Pose2D initialPose = Pose2D(robotTraj.x, robotTraj.y, robotTraj.angle);
            poseGraph.addPriorNode(initialPose);
            std::cout << "Adding the first node to the pose grpah" << std::endl;
            std::cout << robotTraj.x << "--" << robotTraj.y << "--" << robotTraj.angle << std::endl;
        }

        Pose2D odomPose = Pose2D();
        if (poseGraph.graphOptimized)
        {
            Pose2D correctingPose = poseGraph.getCorrectingPose();
            float x = robotTraj.x + correctingPose.x;
            float y = robotTraj.y + correctingPose.y;
            float angle = robotTraj.angle + correctingPose.theta;
            odomPose = Pose2D(x, y, angle);
            poseGraph.addOdomNode(odomPose);
        } else {
            // Adding consecutive nodes to the pose graph with odometry constraints
            odomPose = Pose2D(robotTraj.x, robotTraj.y, robotTraj.angle);
            poseGraph.addOdomNode(odomPose);
        }
        // Loop closing constraint
        if (PoseGraph::getNodesCount() == 263){
            std::cout << "Loop closing between nodeId=9 and nodeId=262" << std::endl;
            std::cout << "Loop closing constraint; [x, y, thta] = [-0.905603, -0.340513, 0.185957]" << std::endl;
            // Relative Transformation between two poses from loop closing detection.
            uint nodeIdi = 9;
            uint nodeIdj = 262;
            Pose2D observation = Pose2D(-0.905603, -0.340513, 0.185957);
            poseGraph.connectLoopClosingNodes(nodeIdi, nodeIdj, observation);

            // We only optimize the graph between two loop closing nodes.
            GraphNode& from = poseGraph.graphNodes[nodeIdi];
            GraphNode& to = poseGraph.graphNodes[nodeIdj];
            poseGraph.optimizeGraph(from, to);
        }

        // Display the robot image at the current pose
        float scale = 22.0;
        QGraphicsPixmapItem* robotItem = new QGraphicsPixmapItem(robotImage.scaledToWidth(20));  // Adjust the size as needed
        robotItem->setPos(odomPose.x * scale, odomPose.y * scale);
        robotItem->setRotation(odomPose.theta * 180 / M_PI);  // Convert radians to degrees
        scene.addItem(robotItem);

        // Add a delay for animation (adjust as needed)
        if (PoseGraph::getNodesCount() == 263)
        {
            while (timer.elapsed() < 3000) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
                // Display the robot trajectory after optimization
                for (uint i = 9; i < poseGraph.graphNodes.size(); i++) {
                    QPixmap robotImage("/home/khorshidi/git/pose_graph_optimization/src/main_exe/robot_image2.png");  // Replace with your image file path
                    QGraphicsPixmapItem* robotItem = new QGraphicsPixmapItem(robotImage.scaledToWidth(20));  // Adjust the size as needed
                    robotItem->setPos(poseGraph.graphNodes[i].nodePose.x * scale, poseGraph.graphNodes[i].nodePose.y * scale);
                    robotItem->setRotation(poseGraph.graphNodes[i].nodePose.theta * 180 / M_PI);  // Convert radians to degrees
                    scene.addItem(robotItem);
                }
            }
        } else {
            while (timer.elapsed() < 10) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            }
        }
        timer.restart();  // Restart the timer for the next iteration
    }

    // Start the Qt event loop
    return a.exec();
}
