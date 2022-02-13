cmake_minimum_required(VERSION 3.0.2)
project(current_state_publisher)

find_package(catkin_simple REQUIRED)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

cs_add_library(odom_tf src/OdomTf.cpp)   

cs_add_executable(current_state_publisher_demo src/odom_tf_main.cpp)

target_link_libraries(current_state_publisher_demo current_state_publisher)

cs_install()
cs_export()
