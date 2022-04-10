#include "lidar_imu_sync/lidar_imu_sync.hpp"
#include "lidar_imu_sync/konitiki_SO3_test.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_imu_sync");
    ros::NodeHandle nh, nh_private("~");

    ROS_INFO("HELLO ROS, This is lidar imu sync");

    // LidarIMUSync lidar_imu_sync(nh, nh_private);

    std::string bag_path = "/home/gxf/multi-sensor-fusion/calib_ws/data/Garage-01.bag";
    std::string imu_topic_name = "/imu1/data_sync";
    SO3Test SO3_test;
    SO3_test.ReadImuData( bag_path,  imu_topic_name);
    SO3_test.Test();

    // ros::spin();
    return 0;
}