#ifndef C5DCBC8F_1117_47C3_919F_9581F3F08C26
#define C5DCBC8F_1117_47C3_919F_9581F3F08C26

#include </home/gxf/multi-sensor-fusion/calib_ws/src/ad_sensor_fusion/src/lidar_imu_sync/third_party/Kontiki/include/kontiki/sensors/constant_bias_imu.h>
#include <kontiki/sensors/vlp16_lidar.h>
#include <kontiki/trajectory_estimator.h>
#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/measurements/accelerometer_measurement.h>
#include <kontiki/measurements/lidar_surfel_point.h>
#include <kontiki/measurements/orientation_measurement.h>
#include <kontiki/measurements/position_measurement.h>
#include <fstream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "lidar_imu_sync.hpp"

class TrajectoryManager
{
    using IMUSeneor = kontiki::sensors::ConstantBiasImu;
    using SO3TrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformR3SplineTrajectory>;
    using R3TrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformR3SplineTrajectory>;
    using SplitTrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::SplitTrajectory>;

    using GyroMeasurement = kontiki::measurements::GyroscopeMeasurement<IMUSensor>; // IMU的角速度观测
    using OrientationMeasurement = kontiki::measurements::OrientationMeasurement;

public:
    typedef std::shared_ptr<TrajectoryManager> Ptr;
    using Result = std::unique_ptr<kontiki::trajectories::TrajectoryEvaluation<double>>;

    explicit TrajectoryManager(double start_time,double end_time,
                                double knot_dist = 0.02,
                                double time_offset_padding =0)
            :time_offset_padding_(time_offset_padding),
            imu_(std::make_shared<IMUSeneor>())
    {
        assert(knot_dist>0);
        double traj_start_time = start_time-time_offset_padding;
        double traj_end_time = end_time + time_offset_padding;
        traj_ = std::make_shared<kontiki::trajectories::SplitTrajectory>
                (knot_dist,knot_dist,traj_start_time,traj_start_time);
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        EIgen::Vector3d p0(0,0,0);
        traj_->SO3Spline()->ExtendTo(end_time,q0);
        traj_->R3Spline()->ExtendTo(end_time,p0);
    }
                                

    void initialSO3TrajWithGyro()
    {
        std::shared_ptr<SO3TrajEstimator> estimate_SO3; 
        estimate_SO3= std::make_shared<SO3TrajEstimator>(traj_->SO3Spline()); // SO3样条
        addGyroscopeMeasurements(estimate_SO3);

        // 固定初始值
        double t0 = traj_->SO3Splne()->MinTime();
        Eigen::AngleAxised rotation_vec(0.0,Eigen::Vector3d(0,0,1));
        Eigen::Quaterniond q0(1,0,0,0);
        auto m_q0 = std::make_shared<OrientationMeasurement>(t0,q0,gyro_weight_);
        estimate_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

        // 四元数求解
        ceres::Solver::Summary summary = estimate_SO3->Solve(30,false);
        std::cout<<summary.BriefReport()<<std::endl;
    }
private:
    std::shared_ptr<kontiki::trajectories::SplitTrajectory> traj_; // 定义一个轨迹指针
    std::shared_ptr<kontiki::sensors::ConstantBiasImu> imu_;
    template<typename TrajectoryModel>
    void addGyroscopeMeasurements(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator)
    {
        gyro_list_.clear();
        double weight = gyro_weight_;
        const double min_time = estimator->trajectory()->MinTime();
        const double max_time = estimator->trajectory()->MiaxTime();
        for(auto& v:imu_data_)
        {
            if(v.stamp<min_time||v.stamp>=max_time) {continue;}
            auto mg = std::make_shared<GyroMeasurement>(imu_,v.stamp,v.gyr,weight);
            gyro_list_.push_back(mg);
            estimator->template AddMeasurement<GyroMeasurement>(mg);
        }   
    }
    std::vector< std::shared_ptr<GyroMeasurement>>  gyro_list_; // 保存指定时间段内的gyro值

    double time_offset_padding_;
    double gyro_weight_{1.0};
    std::vector<ImuData> imu_data_;
    std::vector<GyroMeasurement> gyro_list_;
};





#endif /* C5DCBC8F_1117_47C3_919F_9581F3F08C26 */
