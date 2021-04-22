//
// Created by zlc on 2021/4/18.
//

#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"


namespace lidar_localization
{

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic)
{
    // subscriber 使用智能指针完成订阅函数初始化注册，绑定各自的回调函数
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
    gnss_pub_ptr_  = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "velo_link", 100);

    // 点云补畸变
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

// 点云数据预处理总流程
bool DataPretreatFlow::Run()
{
    if (!ReadData())
        return false;

    if (!InitCalibration())
        return false;

    if (!InitGNSS())
        return false;

    while (HasData())
    {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

// 读取数据
bool DataPretreatFlow::ReadData()
{
    // 以点云数据的时间为准，进行其他三种数据同步
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    // 未同步的数据
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
bool DataPretreatFlow::InitCalibration()
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
bool DataPretreatFlow::InitGNSS()
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
bool DataPretreatFlow::HasData()
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
bool DataPretreatFlow::ValidData()
{
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_imu_time < -0.05 || diff_velocity_time<-0.05 || diff_gnss_time < -0.05)
    {
        cloud_data_buff_.pop_front();
        return false;
    }
    // 当点云的时间戳比imu的时间大的时候，说明当前的imu gnss数据时间早，则删除imu和gnss队列中的第一帧数据
    if (diff_imu_time > 0.05)
    {
        imu_data_buff_.pop_front();
        return false;
    }
    // 轮速计速度
    if (diff_velocity_time > 0.05)
    {
        velocity_data_buff_.pop_front();
        return false;
    }
    // GPS数据
    if (diff_gnss_time > 0.05)
    {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

// 第6个函数：数据转换，将组合导航（到map的）位姿转换为lidar（到map的）位姿，点云不畸变需要知道车的运动速度
// 所以包含如何得到lidar坐标系下的车速，
// 最后进行点云补畸变
bool DataPretreatFlow::TransformData()
{
    gnss_pose_ = Eigen::Matrix4f::Identity();

    // GPS的东北天坐标系
    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;        // lidar_to_map = imu_to_map * lidar_to_imu

    // 得到当前的lidar坐标系下的速度   imu到lidar速度
    current_velocity_data_.TransformCoordinate(lidar_to_imu_.inverse());

    // 进行最重要的点云不畸变                扫描周期100ms，即扫描频率为10Hz
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}


// 第7个函数：发布预处理之后的点云数据
bool DataPretreatFlow::PublishData()
{
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);   // 发布点云里程计
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);                           // 发布GNSS组合导航

    return true;
}


}

