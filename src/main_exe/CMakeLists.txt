cmake_minimum_required(VERSION 3.10)
project(RobotVisualization)

# Find OpenQt
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Add source files
set(SOURCES
    main.cpp
)

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link the Qt5 libraries
target_link_libraries(${PROJECT_NAME} 
    UtilLibrary
    ProgramLibrary
    Qt5::Core Qt5::Widgets)