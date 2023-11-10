cmake_minimum_required(VERSION 3.16)
project(MyProject VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

# Set build type.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set additional flags
set(CMAKE_CXX_FLAGES "Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -o0")
set(CMAKE_CXX_FLAGS_RELEASE "-o3")

# Adding our library
add_library(
    say_hello  STATIC # default is static, other arguments: SHARED, MODULE
    hello.hpp
    hello.cpp
    )

# Show some messages (WARNING, FATAL_ERROR)
message(STATUS "Cmake type is: " ${CMAKE_BUILD_TYPE})


# Add main.cpp file of project *root* directory as source file
set(SOURCE_FILE main.cpp)
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(${PROJECT_NAME} ${SOURCE_FILE})

# Linking the library to the main executable.
# target_link_libraries(<The library we are linking into> <linker_mode> <our_lib>)
target_link_libraries(${PROJECT_NAME} PRIVATE say_hello)