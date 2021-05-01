cmake_minimum_required(VERSION 3.0.2)
project(sensor_read)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        serial
        message_generation
        )

add_message_files(
        FILES
        MyoData.msg
        MyoDataSingle.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp serial message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/sensor_read_node.cpp
        )

add_dependencies(${PROJECT_NAME}_node
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES})