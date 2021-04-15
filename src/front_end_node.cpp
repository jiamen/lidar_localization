//
// Created by zlc on 2021/4/1.
//


#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"


#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/front_end/front_end_flow.hpp"


using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;


bool save_map_callback(saveMap::Request &request, saveMap::Response &response)
{
    response.succeed = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublishGlobalMap();
    LOG(INFO) << "response.succeed: " << response.succeed;
    return response.succeed;
}



int main(int argc, char* *argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;      // 设置日志消息除了日志文件之外是否去标准输出


    // 2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;


    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);


    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}

