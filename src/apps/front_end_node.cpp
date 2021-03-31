//
// Created by zlc on 2021/3/28.
//

#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

using namespace lidar_localization;


int main(int argc, char* *argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;




    return 0;
}


