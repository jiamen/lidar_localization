
cmake_minimum_required(VERSION 2.8.3)

project(lidar_localization)

set(CMAKE_BUIlD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    pcl_ros
    geometry_msgs
    tf
    eigen_conversions
    message_generation
    std_srvs        # （参数为空的服务）
)

add_service_files(
    FILES
    saveMap.srv
    optimizeMap.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

set(ALL_TARGET_LIBRARIES "")

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )
# include(cmake/eigen.cmake)
include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
# 用于执行地理、UTM、UPS、MGRS、地心和局部笛卡尔坐标之间的转换，用于重力（例如EGM2008）、大地水准面高度和地磁场（例如，WMM2020）计算，以及用于解决测地线问题。
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)
include(cmake/global_defination.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTEND_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})






