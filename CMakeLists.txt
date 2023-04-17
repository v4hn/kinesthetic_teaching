cmake_minimum_required(VERSION 3.0.2)
project(kinesthetic_teaching)

find_package(catkin REQUIRED COMPONENTS
    rospy
    moveit_commander
    trajectory_msgs
    std_msgs
    sensor_msgs
    # roscpp
    # moveit_ros_planning
)

catkin_package()

catkin_python_setup()

include_directories()

