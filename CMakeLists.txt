cmake_minimum_required(VERSION 3.10)



# set the project name
project(Filters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Use the package PkgConfig to detect GTK+ headers/library files
FIND_PACKAGE(PkgConfig REQUIRED)

SET(CMAKE_CXX_FLAGS "-O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR})
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR})


# add the executable
add_executable(Filters main.cpp model.cpp filters.cpp)

# Link the target to the GTK+ libraries
TARGET_LINK_LIBRARIES(Filters armadillo)
target_compile_options(Filters PRIVATE  -rdynamic  -export-dynamic)
