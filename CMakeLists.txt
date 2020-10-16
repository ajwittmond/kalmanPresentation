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

find_package(Boost 1.72.0)
find_package(Backtrace)
# Setup CMake to use GTK+, tell the compiler where to look for headers
# and to the linker where to look for libraries
INCLUDE_DIRECTORIES(${Backtrace_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GTK3_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
LINK_DIRECTORIES(${GTK3_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR})

# add the executable
add_executable(Filters main.cpp simulation.cpp simulationWindow.cpp filters.cpp models.cpp)

# Link the target to the GTK+ libraries
TARGET_LINK_LIBRARIES(Filters armadillo dl backtrace ${BACKTRACE_LIBRARIES} ${GTK3_LIBRARIES})
target_compile_options(Filters PRIVATE -g -ggdb -O0 -rdynamic  -export-dynamic  )
