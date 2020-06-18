
cmake_minimum_required(VERSION 2.8)

project(angle_utility)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}"-std=c++14")

find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        sensor_msgs
        tf
)

catkin_package(

        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        geometry_msgs
        tf
        
)

set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_PATH} ${Boost_INCLUDE_DIR} include)


add_executable(angle_checker
        src/angle_checker.cpp
        )
target_link_libraries(angle_checker
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

add_executable(angle_gazebo_checker
        src/angle_gazebo_checker.cpp
        )
target_link_libraries(angle_gazebo_checker
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
#add_dependencies(angle_checker
#        ${PROJECT_NAME}_generate_messages_cpp
#        )

add_executable(fake_info
        src/fake_camera_info.cc
        )
target_link_libraries(fake_info
        ${catkin_LIBRARIES}

        )