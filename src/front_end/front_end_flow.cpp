//
// Created by zlc on 2021/4/1.
//

#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"                // 涉及到文件操作
#include "lidar_localization/global_defination/global_defination.h"


namespace lidar_localization
{

    FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh)
    {
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);       // 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
        imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
        velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
        gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
        lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
        // 最后显示的两条轨迹就来自于这两个发布
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
        gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);

        front_end_ptr_ = std::make_shared<FrontEnd>();


        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());
        current_scan_ptr_.reset(new CloudData::CLOUD());
    }

    // 函数运行总流程
    bool FrontEndFlow::Run()
    {
        // 读取订阅数据，包括点云、imu、轮速计速度、gps 4种数据
        if (!ReadData())
            return false;

        // 初始化校准数据，得到lidar_to_imu的变换矩阵
        if (!InitCalibration())
            return false;

        // 初始化GPS
        if (!InitGNSS())
            return false;

        while (HasData())
        {
            if (!ValidData())
                continue;

            // 更新GPS里程计
            UpdateGNSSOdometry();
            if (UpdateLaserOdometry())      // 更新激光雷达里程计
            {
                // 发布点云数据
                PublishData();
                // 保存轨迹
                if(!SaveTrajectory())
                {
                    break;
                }
            }
        }

        return true;
    }

    bool FrontEndFlow::ReadData()
    {
        // 以点云数据的时间为准，进行其他三种数据同步
        cloud_sub_ptr_->ParseData(cloud_data_buff_);

        static std::deque<IMUData> unsynced_imu_;
        static std::deque<VelocityData> unsynced_velocity_;
        static std::deque<GNSSData> unsynced_gnss_;

        imu_sub_ptr_->ParseData(unsynced_imu_);
        velocity_sub_ptr_->ParseData(unsynced_velocity_);
        gnss_sub_ptr_->ParseData(unsynced_gnss_);

        // 点云队列中没有数据则直接返回false
        if (0 == cloud_data_buff_.size())
            return false;

        // 点云队列中有数据则进行3种数据同步
        double cloud_time = cloud_data_buff_.front().time;
        bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
        bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
        bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

        static bool sensor_inited = false;
        if (!sensor_inited)
        {
            // 只要有gps，imu，轮速计velocity一个无效，
            if (!valid_imu || !valid_velocity || !valid_gnss)
            {
                cloud_data_buff_.pop_front();
                return false;
            }
            sensor_inited = true;
        }

        return true;
    }

    // 第2个调用函数：初始化校准，得到lidar_to_imu的变换矩阵，在里程计初始部分会使用到
    bool FrontEndFlow::InitCalibration()
    {
        static bool calibration_received = false;

        if (!calibration_received)
        {
            if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_))
            {
                calibration_received = true;
            }
        }

        return calibration_received;
    }

    // 第3个调用函数：初始化GPS, 设置gnss的原点
    bool FrontEndFlow::InitGNSS()
    {
        static bool gnss_inited = false;

        if (!gnss_inited)
        {
            GNSSData gnss_data = gnss_data_buff_.front();
            gnss_data.InitOriginPosition();         // 初始化GPS, 设置gnss的原点
            gnss_inited = true;
        }

        return gnss_inited;
    }


    // 第4个调用函数：判断4个数据队列中有误数据
    bool FrontEndFlow::HasData()
    {
        if (0 == cloud_data_buff_.size())
            return false;

        if (0 == imu_data_buff_.size())
            return false;

        if (0 == gnss_data_buff_.size())
            return false;

        if (0 == velocity_data_buff_.size())
            return false;

        return true;
    }


    // 第5个调用函数：有数据的情况下，判断数据是否有效
    // 判断当前取出的点云数据，imu数据，gnss数据是不是处于同一时刻，即对数据进行时间同步。
    bool FrontEndFlow::ValidData()
    {
        current_cloud_data_ = cloud_data_buff_.front();
        current_imu_data_ = imu_data_buff_.front();
        current_velocity_data_ = velocity_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();

        double d_time = current_cloud_data_.time - current_imu_data_.time;
        if (d_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }
        // 当点云的时间戳比imu的时间大的时候，说明当前的imu gnss数据时间早，则删除imu和gnss队列中的第一帧数据
        if (d_time > 0.05)
        {
            imu_data_buff_.pop_front();
            velocity_data_buff_.pop_front();
            gnss_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

        return true;
    }


// 第6个调用函数：更新GPS里程计
    bool FrontEndFlow::UpdateGNSSOdometry()
    {
        gnss_odometry_ = Eigen::Matrix4f::Identity();

        // 注意：变换矩阵都是当前帧到参考帧的变换矩阵  ， 即 T_rc

        // gnss_odometry_实际上表示的就是ENU坐标系下的变换矩阵,具体说是当前帧到参考帧的变换
        current_gnss_data_.UpdateXYZ();
        // 东北天ENU坐标系下的平移， 将gnss经纬度海拔转化为ENU坐标
        gnss_odometry_(0,3) = current_gnss_data_.local_E;                                   // gps测量的数据也是相对于ENU坐标系
        gnss_odometry_(1,3) = current_gnss_data_.local_N;
        gnss_odometry_(2,3) = current_gnss_data_.local_U;
        // 得到lidar到map（ENU）的变换
        gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();  // imu测量的数据是imu坐标系相对于ENU坐标系的
        // gnss_odometry_是imu坐标系相对于ENU坐标系的

        gnss_odometry_ *= lidar_to_imu_;
        // lidar_to_ENU=imu_to_ENU*lidar_to_imu

        return true;
    }


// 第7个调用函数：更新雷达点云里程计
    bool FrontEndFlow::UpdateLaserOdometry()
    {
        static bool front_end_pose_inited = false;

        if ( !front_end_pose_inited )
        {
            front_end_pose_inited = true;
            front_end_ptr_->SetInitPose(gnss_odometry_);        // 当第一帧点云匹配的时候，需要设置初始位姿
            return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
        }

        laser_odometry_ = Eigen::Matrix4f::Identity();
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }


    // 第8个调用函数：发布数据
    bool FrontEndFlow::PublishData()
    {
        gnss_pub_ptr_->Publish(gnss_odometry_);             // 发布gnss里程计
        laser_odom_pub_ptr_->Publish(laser_odometry_);      // 发布雷达点云里程计

        front_end_ptr_->GetCurrentScan(current_scan_ptr_);
        cloud_pub_ptr_->Publish(current_scan_ptr_);

        if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
            local_map_pub_ptr_->Publish(local_map_ptr_);

        return true;
    }


    // 第9个调用函数：保存轨迹
    bool FrontEndFlow::SaveTrajectory()
    {
        static std::ofstream ground_truth, laser_odom;
        static bool is_file_created = false;
        if (!is_file_created)
        {
            if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
                return false;
            if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
                return false;
            if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
                return false;
            is_file_created = true;
        }

        for (int i = 0; i < 3; ++ i)
        {
            for (int j = 0; j < 4; ++ j)
            {
                ground_truth << gnss_odometry_(i, j);
                laser_odom << laser_odometry_(i, j);

                if (i == 2 && j == 3)
                {
                    ground_truth << std::endl;
                    laser_odom << std::endl;
                }
                else
                {
                    ground_truth << " ";
                    laser_odom << " ";
                }
            }
        }

        return true;
    }



    // 下面两个函数在回调函数中
    bool FrontEndFlow::SaveMap()
    {
        LOG(INFO) << "SaveMap!!!!!!!!!!!!!!!!!!!!!!!";
        return front_end_ptr_->SaveMap();
    }

    bool FrontEndFlow::PublishGlobalMap()
    {
        if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_))
        {
            global_map_pub_ptr_->Publish(global_map_ptr_);
            global_map_ptr_.reset(new CloudData::CLOUD());
        }

        return true;
    }

}
