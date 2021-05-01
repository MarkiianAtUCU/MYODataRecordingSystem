cmake_minimum_required(VERSION 3.0.2)
project(cv_arm_detection)

add_compile_options(-std=c++17)
add_definitions(-Wall)


find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_geometry
        image_transport
        roscpp
        std_msgs
        tf2_geometry_msgs
        geometry_msgs

        message_generation
        dynamic_reconfigure
        )

generate_dynamic_reconfigure_options(
        cfg/ArmDetection.cfg
)

catkin_package()

find_package(OpenCV COMPONENTS highgui features2d calib3d REQUIRED)
set(OPENCV_DEPENDENCIES highgui features2d calib3d)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/arm_detection_node.cpp
        src/ArmDetection.cpp

        include/cv_arm_detection/ArmDetection.h
        src/ocv_utils.cpp include/ocv_utils.h)

add_dependencies(${PROJECT_NAME}_node
        ${${PROJECT_NAME}_node_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBRARIES})