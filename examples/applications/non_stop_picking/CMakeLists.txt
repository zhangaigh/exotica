cmake_minimum_required(VERSION 2.8.3)
project(non_stop_picking)

add_compile_options(-std=c++11)
find_package(OMPL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  exotica time_indexed_rrt_connect ik_solver
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES non_stop_picking
  CATKIN_DEPENDS exotica time_indexed_rrt_connect ik_solver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/NonStopPicking.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OMPL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/non_stop_picking_node.cpp
  src/NonStopPicking.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${OMPL_LIBRARIES}
)