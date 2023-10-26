cmake_minimum_required(VERSION 3.16)

if (${CMAKE_VERSION} VERSION_LESS 3.16)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
    message("Using your cmake(less than 3.16)")
endif()

project(pure_pursuit_controller DESCRIPTION "a pure pursuit example")

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    "src/pure_pursuit_node.cpp"
    "src/math_tool.cpp"
)

add_executable(${PROJECT_NAME}_node
    src/pure_pursuit.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)