cmake_minimum_required(VERSION 3.0.2)
project(dataset_generator)

add_compile_options(-std=c++17)
add_definitions(-Wall)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        sensor_read
        )


catkin_package(
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/dataset_generator_node.cpp
        include/dataset_generator/DatasetGenerator.cpp
        include/dataset_generator/DatasetGenerator.h
        )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES})

add_executable(fake_data_generation_node
        src/fake_data_generation_node.cpp
        )

target_link_libraries(fake_data_generation_node
        ${catkin_LIBRARIES})