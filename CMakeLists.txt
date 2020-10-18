cmake_minimum_required(VERSION 3.0.2)
project(deploy_module)
add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/deploy_manager.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(deploy_manager_node src/deploy_manager_node.cpp)
target_link_libraries(deploy_manager_node deploy_module ${catkin_LIBRARIES})
