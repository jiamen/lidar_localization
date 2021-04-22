cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    pcl_ros
    geometry_msgs
    tf
    eigen_conversions
    message_generation
    std_srvs      # （参数为空的服务）
)

add_service_files(
    FILES
    saveMap.srv
    optimizeMap.srv    # 在后端back_end_node.cpp中会用到
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)
include(cmake/sophus.cmake)

# 添加sophus库
#find_package( Sophus REQUIRED )   ${Sophus_INCLUDE_DIRS}

include_directories(include ${catkin_INCLUDE_DIRS} )
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "/home/zlc/catkin_ws/src/lidar_localization/src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "/home/zlc/catkin_ws/src/lidar_localization/src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "/home/zlc/catkin_ws/src/lidar_localization/third_party/*.cpp")
#file(GLOB_RECURSE ALL_SRCS "*.cpp")
#file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
#file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node  ${Sophus_LIBRARIES} ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} /home/zlc/catkin_ws/devel/lib/liblibGeographiccc.so)

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} /home/zlc/catkin_ws/devel/lib/liblibGeographiccc.so)

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(front_end_node ${Sophus_LIBRARIES} ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} /home/zlc/catkin_ws/devel/lib/liblibGeographiccc.so)

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})