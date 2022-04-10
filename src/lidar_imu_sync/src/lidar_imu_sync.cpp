#include "lidar_imu_sync/lidar_imu_sync.hpp"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

LidarIMUSync::LidarIMUSync(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private)
{
    lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 10, &LidarIMUSync::lidarCallback, this);
    imu_sub_   = nh_.subscribe<sensor_msgs::Imu>("/vectornav/IMU", 100, &LidarIMUSync::imuCallback, this);
    caliber_ = std::make_shared<LidarIMUCalib>();
}

void LidarIMUSync::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 读取激光数据并进行解析
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    LidarData data;
    data.cloud = cloud;
    data.stamp = msg->header.stamp.toSec();
    //将激光雷达数据追加到标定系统中
    caliber_->addLidarData(data);
    // calib
    // 执行标定函数
    Eigen::Vector3d rpy = caliber_->calib();
}

void LidarIMUSync::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
 
    ImuData data;
    // 计算IMU的加速度信息
    data.acc = Eigen::Vector3d( msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
    // IMU的角速度信息
    data.gyr = Eigen::Vector3d( msg->angular_velocity.x,
                                msg->angular_velocity.y,
                                msg->angular_velocity.z);
    // IMU的姿态信息
    data.rot = Eigen::Quaterniond(  msg->orientation.w,
                                    msg->orientation.x,
                                    msg->orientation.y,
                                    msg->orientation.z);
    data.stamp = msg->header.stamp.toSec();
    //将IMU解析后的加速度、角速度、姿态信息追加到标定数据中
    caliber_->addImuData(data);
    //将追加后的数据从imu队列中删除掉
}
