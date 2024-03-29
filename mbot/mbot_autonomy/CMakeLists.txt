cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mbot_autonomy)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)
find_package(GTK2 2.6 REQUIRED gtk)

find_library(common_utils REQUIRED)
find_library(mbot_lcm_msgs REQUIRED)
find_library(mbot_drivers REQUIRED)

include(${LCM_USE_FILE})


# MOTION CONTROLLER
add_executable(motion_controller src/mbot/motion_controller.cpp)

target_link_libraries(motion_controller
  common_utils
  lcm
  mbot_lcm_msgs-cpp
)
target_include_directories(motion_controller PRIVATE
  include
)

# SLAM
add_executable(slam src/slam/slam_main.cpp
  src/slam/action_model.cpp
  src/slam/mapping.cpp
  src/slam/moving_laser_scan.cpp
  src/slam/occupancy_grid.cpp
  src/slam/particle_filter.cpp
  src/slam/sensor_model.cpp
  src/slam/slam.cpp
  # src/utils/pose_trace.cpp
)
target_link_libraries(slam
  ${CMAKE_THREAD_LIBS_INIT}
  common_utils
  lcm
  mbot_lcm_msgs-cpp
)
target_include_directories(slam PRIVATE
  include
)

# EXPLORATION
add_executable(exploration src/planning/exploration_main.cpp
  src/planning/exploration.cpp
  src/planning/motion_planner.cpp 
  src/planning/frontiers.cpp 
  src/slam/occupancy_grid.cpp 
  src/planning/obstacle_distance_grid.cpp
  src/planning/astar.cpp 
)
target_link_libraries(exploration
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(exploration PRIVATE
  include
)

# A* TEST
add_executable(astar_test src/planning/astar_test.cpp
  src/planning/astar.cpp
  src/planning/motion_planner.cpp 
  src/slam/occupancy_grid.cpp 
  src/planning/obstacle_distance_grid.cpp
)

target_link_libraries(astar_test
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(astar_test PRIVATE
  include
)

# OBSTACLE DISTANCE GRID TEST
add_executable(obstacle_distance_grid_test src/planning/obstacle_distance_grid_test.cpp
  src/planning/obstacle_distance_grid.cpp
  src/slam/occupancy_grid.cpp
  src/planning/astar.cpp
)

target_link_libraries(obstacle_distance_grid_test
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(obstacle_distance_grid_test PRIVATE
  include
)

# PLANNING SERVER
add_executable(motion_planning_server src/planning/motion_planner_server_main.cpp
  src/planning/motion_planner.cpp
  src/planning/motion_planner_server.cpp
  src/slam/occupancy_grid.cpp
  src/planning/obstacle_distance_grid.cpp
  src/planning/astar.cpp
)
target_link_libraries(motion_planning_server
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(motion_planning_server PRIVATE
  include
)

# TODO: move out of this project!
# TIMESYNC
add_executable(timesync src/mbot/timesync.cpp
)
target_link_libraries(timesync
  common_utils
  lcm
  mbot_lcm_msgs-cpp
)
target_include_directories(timesync PRIVATE
  include
)

# NOTICE: This is a temporary patch to remove the 
#     gui from this codebase
add_library(autonomy_lib STATIC 
  src/planning/astar.cpp
  src/planning/frontiers.cpp
  src/planning/motion_planner.cpp
  src/planning/obstacle_distance_grid.cpp
  src/slam/occupancy_grid.cpp
)
target_link_libraries(autonomy_lib
  ${CMAKE_THREAD_LIBS_INIT}
  mbot_lcm_msgs-cpp
  common_utils
  lcm
)
target_include_directories(autonomy_lib PUBLIC include)
