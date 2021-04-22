//
// @Description: 前端里程计算法
// Created by zlc on 2021/4/1.
//

#include "lidar_localization/mapping/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"


#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/print_info.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/registration/icp_registration_manual.hpp"
#include "lidar_localization/models/registration/icp_registration.hpp"

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"



namespace lidar_localization
{

FrontEnd::FrontEnd()
        : local_map_ptr_(new CloudData::CLOUD())
{
    LOG(INFO) << "进入FrontEnd()开始初始化：";
    InitWithConfig();
    LOG(INFO) << "进入FrontEnd()结束初始化：";
}

// 初始化总流程
bool FrontEnd::InitWithConfig()
{
    // 在这里读取配置函数
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------前端初始化-------------------" << std::endl;
    InitParam(config_node);         // 初始化参数
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}

// 参数初始化：关键帧的距离、需要维护的局部地图的数量
bool FrontEnd::InitParam(const YAML::Node& config_node)
{
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

// 初始化流程2：确定点云配准方式
bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node)
{
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT")
    {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }
    else if (registration_method == "ICP")
    {
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    }
    else if (registration_method == "ICP_MANUAL")
    {
        registration_ptr = std::make_shared<ICPRegistrationManual>(config_node[registration_method]);
    }
    else
    {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

// 初始化流程3：选择点云滤波方法
bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node)
{
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();     // voxel_filter
    LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_method;

    if (filter_method == "voxel_filter")
    {
        // 选择体素滤波器
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);    // voxel_filter:  local_map:
    }
    else if (filter_method == "no_filter")
    {
        // 无滤波器
        filter_ptr = std::make_shared<NoFilter>();
    }
    else
    {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_method << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

// 更新雷达点云里程计第一帧时，若没有初始位姿，使用ENU坐标系下的位姿，gps提供三轴上的平移，IMU提供三轴上的旋转
bool FrontEnd::SetInitPose(const Eigen::Matrix4f &init_pose)
{
    init_pose_ = init_pose;     // map坐标系下的
    return true;
}


// △△△ 里程计的更新 △△△
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose)
{
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    // 1. 首先去除无效数据
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    // 2. 进行点云滤波
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    // 3. 静态初始值的设定，只设定一次
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity(); // Ti△T_ij=Tj 中的 △T
    static Eigen::Matrix4f last_pose = init_pose_;                  // 保留上一帧的位姿，计算两帧之间相对位姿变换△T
    static Eigen::Matrix4f predict_pose = init_pose_;               // 为NDT、ICP等配准方法提供预测的初值
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;        // 用来判断是否更新地图

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧, 并更新局部地图容器和全局地图容器
    // 3.对第一帧的处理并作为关键帧
    if (local_map_frames_.size() == 0)
    {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);         // 更新关键帧
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 4.ndt匹配，并判断是否可以作为关键帧
    // 不是第一帧，就正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    // 输入待匹配点云  当前输入帧点云到目标点云的预测位姿T     变换后的点云       当前输入帧点云到目标点云的优化变换位姿T
    // result_cloud_ptr: 存储filtered_cloud_ptr经过配准后的点云(由于filtered_cloud_ptr被极大的降采样了,因此这个数据没什么用)
    // 我们只需要配准后得到的变换位姿 current_frame_.pose
    cloud_pose = current_frame_.pose;               // 需要返回的ENU坐标系下的 T_lidar_to_map

    // 更新相邻两帧的相对运动，预测下一帧扫描点云到map（ENU坐标系下的map）的变换
    step_pose = last_pose.inverse() * current_frame_.pose;      // Ti△T_ij=Tj   △T_ij = Ti^(-1)*Tj
    predict_pose = current_frame_.pose * step_pose;     // 成为下次预测的初始位姿T_rc
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) +
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_)
    {
        UpdateWithNewFrame(current_frame_);         // 更新关键帧
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}


// △△△ 关键帧 的更新 △△△
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame)
{
    // 把关键帧点云存储到硬盘里，节省内存
    // std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    // pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已,也就是说不能用
    // key_frame.cloud_data.cloud_ptr = new_key_frame.cloud_data.cloud_ptr
    // 上面这种赋值方法无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    std::cout << "new_key_frame.cloud_data.cloud_ptr： " << new_key_frame.cloud_data.cloud_ptr << std::endl;
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    std::cout << "key_frame.cloud_data.cloud_ptr： " << key_frame.cloud_data.cloud_ptr << std::endl;

    // 地址可能不同，但是指向的是同一片点云
    // 即 *key_frame.cloud_data.cloud_ptr = *new_key_frame.cloud_data.cloud_ptr;

    // 更新局部地图，存放关键帧
    local_map_frames_.push_back(key_frame);
    // 当关键帧数量大于指定的20帧时，弹出最前面的一帧
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_))
    {
        local_map_frames_.pop_front();
    }

    // 根据关键帧构造关键帧组成的局部地图
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    local_map_ptr_.reset(new CloudData::CLOUD());       // 局部地图保存指针
    for (size_t i = 0; i < local_map_frames_.size(); ++ i)
    {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;      // 这里的 += 经过了重载，理解为点云拼接
    }


    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10)
    {
        registration_ptr_->SetInputTarget(local_map_ptr_);          // 设置目标地图点云
    }
    else
    {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);  // 对地图点云进行滤波
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);  // 设置目标地图点云
    }

    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
    // key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    // global_map_frames_.push_back(key_frame);

    return true;
}

}


