# Find Eigen package
find_package(Eigen3 REQUIRED)

# Define the program library
add_library(ProgramLibrary  STATIC
    GraphConstraint.cpp
    PoseGraph.cpp
)
    
# Include directories for UtilLibrary and Eigen
target_include_directories(ProgramLibrary PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils
    ${EIGEN3_INCLUDE_DIR}
)