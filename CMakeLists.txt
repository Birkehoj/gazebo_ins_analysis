cmake_minimum_required(VERSION 3.26)
project(gazebo_pose_recorder)

set(CMAKE_CXX_STANDARD 20)

set(ENV{Qt5_DIR} /opt/homebrew/Cellar/qt@5/5.15.10_1)
find_package(Qt5 COMPONENTS Core REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS all)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

message(STATUS "Gazebo Plugin version: ${GZ-PHYSICS_INCLUDE_DIRS}")
find_package(gz-physics6 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics6_VERSION_MAJOR})

add_library(InsCsvLogger SHARED
    InsCsvLogger.cpp
    InsCsvLogger.h)

target_link_libraries(InsCsvLogger
    PRIVATE
    gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

target_compile_definitions(InsCsvLogger
    PRIVATE
    DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)