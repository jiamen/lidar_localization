//
// Created by zlc on 2021/4/15.
//

#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"


using namespace lidar_localization;


int main(int argc, char* *argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    // 初始化ROS节点，该初始化的init函数包含三个参数，前两个参数是命令行或者launch文件输入的参数，可以用来完成命名重映射等功能；
    // 第三个参数定义了本XXX_node节点的名称，而且该名称在运行的ROS中必须独一无二，不允许同时存在相同名称的两个节点。
    ros::init(argc, argv, "data_pretreat_node");

    // 创建节点句柄，方便对节点资源的使用和管理。
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    // 代表将名字为cloud_topic的参数值赋值给中间的cloud_topic变量，没有参数值时默认将/synced_cloud赋值给cloud_topic


    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

    ros::Rate rate(100);
    while ( ros::ok() )
    {
        ros::spinOnce();    // 用来处理节点订阅话题的所有回调函数

        data_pretreat_flow_ptr->Run();

        rate.sleep();       // 一个周期的工作完成，可以让节点休息一段时间，调用休眠函数，节点进入休眠状态，10Hz的休眠时间，节点休眠100ms后又会开始一个周期的循环工作
    }

    return 0;
}


