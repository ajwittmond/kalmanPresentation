cmake_minimum_required(VERSION 3.10)



# set the project name
project(Filters)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Use the package PkgConfig to detect GTK+ headers/library files
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(GTK3 REQUIRED gtkmm-3.0)

SET(CMAKE_CXX_FLAGS "-O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Setup CMake to use GTK+, tell the compiler where to look for headers
# and to the linker where to look for libraries
INCLUDE_DIRECTORIES( ${GTK3_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
LINK_DIRECTORIES(${GTK3_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR})

# add the executable
add_executable(Filters main.cpp simulation.cpp simulationWindow.cpp filters.cpp models.cpp ini_parser.cpp)

# Link the target to the GTK+ libraries
TARGET_LINK_LIBRARIES(Filters armadillo  ${GTK3_LIBRARIES})
target_compile_options(Filters PRIVATE  -rdynamic  -export-dynamic  )
