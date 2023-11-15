# Define the utility library
add_library(UtilLibrary
    Pose2D.cpp
)

# Include the headers for utility library
target_include_directories(UtilLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})