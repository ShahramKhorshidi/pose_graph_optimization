# 2D Pose Graph Optimization

## Introduction

This is a C++ implementation of an efficient 2D pose graph optimization, by exploiting the sparsity in the graph structure, as outlined in [1, 2]

## Installation

### Dependencies

Eigen, for optimization. <br>
Qt Creator, optional, if you want to run the example.
```
sudo apt-get update
sudo apt-get install qtcreator
```

### Source Code

The source code is hosted on GitHub: [ShahramKhorshidi/pose_graph_optimization](https://github.com/ShahramKhorshidi/pose_graph_optimization)
```
# Clone pose_graph_optimization
git clone git@github.com:ShahramKhorshidi/pose_graph_optimization.git
```

### Build
```
cd pose_graph_optimization
mkdir build && cd build/ 
cmake ..
make
```
### Quick Start
You can run the example with recorded trajectory of a wheeled robot.
Before running the example change the main.cpp at line 24, adapting the path to your local repository.
```
cd pose_graph_optimization/build/example
./RobotVisualization
```
## Framework


## References
[1] G. Grisetti, R. Kümmerle, C. Stachniss and W. Burgard, "A Tutorial on Graph-Based SLAM," in IEEE Intelligent Transportation Systems Magazine, 2010.

[2] K. Konolige, G. Grisetti, R. Kümmerle, W. Burgard, B. Limketkai and R. Vincent, "Efficient Sparse Pose Adjustment for 2D mapping," 2010 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2010.
