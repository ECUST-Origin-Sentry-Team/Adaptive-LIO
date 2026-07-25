// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <deque>
#include <algorithm>
#include <condition_variable>

// ros2 lib
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <pcl/common/transforms.h>

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert2.h"
#include "lio/lidarodom.h"

#include "scantext_module/Mapping.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

nav_msgs::msg::Path laserOdoPath;

DEFINE_string(config_file, "", "Path to adaptive_lio config YAML.");

zjloc::lidarodom_m *lio;
zjloc::CloudConvert2 *convert;
std::shared_ptr<scantext::MappingCore> scantext_mapping;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;
std::shared_ptr<class ImuOdomFusion> imu_odom_fusion;
std::shared_ptr<class DualImuCenterOdom> dual_imu_center_odom;
double gnorm = 1.0;
// rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_repub;

#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "Log/" + name))

inline rclcpp::Time get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}

inline Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
    tf2::Quaternion q;
    q.setRPY(rpy.x(), rpy.y(), rpy.z());
    return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized();
}

class FrameStampSynchronizer
{
public:
    void Record(double sensor_stamp, const rclcpp::Time &output_stamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.push_back({sensor_stamp, output_stamp});
        while (entries_.size() > 32)
        {
            entries_.pop_front();
        }
        cv_.notify_all();
    }

    bool WaitLookup(
        double sensor_stamp, rclcpp::Time &output_stamp, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto find_entry = [&]() {
            return std::find_if(
                entries_.begin(), entries_.end(),
                [sensor_stamp](const Entry &entry) {
                    return std::abs(sensor_stamp - entry.sensor_stamp) <= 1e-4;
                });
        };

        if (!cv_.wait_for(lock, timeout, [&]() { return find_entry() != entries_.end(); }))
        {
            return false;
        }

        const auto entry = find_entry();
        output_stamp = entry->output_stamp;
        entries_.erase(entry);
        return true;
    }

private:
    struct Entry
    {
        double sensor_stamp;
        rclcpp::Time output_stamp;
    };

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Entry> entries_;
};

class DualImuCenterOdom
{
public:
    struct Options
    {
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        Eigen::Vector3d sensor_to_base_translation = Eigen::Vector3d::Zero();
        Eigen::Quaterniond sensor_to_base_rotation = Eigen::Quaterniond::Identity();
        Eigen::Matrix3d primary_imu_to_base_rotation = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d secondary_imu_to_base_rotation = Eigen::Matrix3d::Identity();
        double secondary_time_offset = 0.0;
        double sync_tolerance = 0.003;
        double acceleration_lpf_tau = 0.06;
        double max_speed = 35.0;
        double gravity = 9.81;
        bool estimate_initial_bias = true;
        bool restamp_to_now = true;
    };

    DualImuCenterOdom(
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &publisher,
        const rclcpp::Clock::SharedPtr &clock,
        const Options &options)
        : publisher_(publisher),
          clock_(clock),
          options_(options)
    {
        options_.sensor_to_base_rotation.normalize();
        options_.sync_tolerance = std::max(1e-5, options_.sync_tolerance);
        options_.acceleration_lpf_tau = std::max(0.0, options_.acceleration_lpf_tau);
        options_.max_speed = std::max(0.1, options_.max_speed);
    }

    void AddPrimary(const IMUPtr &imu)
    {
        AddImu(imu, true);
    }

    void AddSecondary(const IMUPtr &imu)
    {
        AddImu(imu, false);
    }

    void OnLioPose(
        const SE3 &pose,
        double sensor_stamp,
        const rclcpp::Time &output_stamp)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        Eigen::Quaterniond q_odom_sensor(pose.rotationMatrix());
        q_odom_sensor.normalize();
        orientation_ =
            (q_odom_sensor * options_.sensor_to_base_rotation).normalized();
        position_ =
            pose.translation() +
            q_odom_sensor.toRotationMatrix() * options_.sensor_to_base_translation;

        if (!initialized_)
        {
            InitializeLocked(sensor_stamp);
        }

        PublishLocked(output_stamp);
    }

private:
    struct Sample
    {
        double stamp = 0.0;
        Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    };

    void AddImu(const IMUPtr &imu, bool primary)
    {
        if (!imu)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        Sample sample;
        sample.stamp = imu->timestamp_ + (primary ? 0.0 : options_.secondary_time_offset);
        const Eigen::Matrix3d &rotation =
            primary ? options_.primary_imu_to_base_rotation
                    : options_.secondary_imu_to_base_rotation;
        sample.gyro = rotation * imu->gyro_;
        sample.acc = rotation * imu->acce_;

        auto &queue = primary ? primary_queue_ : secondary_queue_;
        queue.push_back(sample);
        while (queue.size() > kMaxQueueSize)
        {
            queue.pop_front();
        }

        MatchAndProcessLocked();
    }

    void MatchAndProcessLocked()
    {
        while (!primary_queue_.empty() && !secondary_queue_.empty())
        {
            const double skew =
                primary_queue_.front().stamp - secondary_queue_.front().stamp;
            if (std::abs(skew) <= options_.sync_tolerance)
            {
                const Sample primary = primary_queue_.front();
                const Sample secondary = secondary_queue_.front();
                primary_queue_.pop_front();
                secondary_queue_.pop_front();
                ProcessAverageLocked(primary, secondary);
                return;
            }

            if (skew < 0.0)
            {
                primary_queue_.pop_front();
            }
            else
            {
                secondary_queue_.pop_front();
            }
        }
    }

    void ProcessAverageLocked(const Sample &primary, const Sample &secondary)
    {
        const double stamp = primary.stamp;
        const Eigen::Vector3d gyro = 0.5 * (primary.gyro + secondary.gyro);
        const Eigen::Vector3d acc = 0.5 * (primary.acc + secondary.acc);
        latest_gyro_ = gyro;
        latest_pair_stamp_ = stamp;
        has_pair_stamp_ = true;

        if (!initialized_)
        {
            preinit_gyro_sum_ += gyro;
            preinit_acc_sum_ += acc;
            ++preinit_count_;
            return;
        }

        const double dt = stamp - last_imu_stamp_;
        last_imu_stamp_ = stamp;
        if (!(dt > 0.0) || dt > kMaxImuDt)
        {
            has_acc_lpf_ = false;
            return;
        }

        const Eigen::Vector3d omega = gyro - gyro_bias_;
        const double angle = omega.norm() * dt;
        if (angle > 1e-12)
        {
            const Eigen::Quaterniond dq(
                Eigen::AngleAxisd(angle, omega.normalized()));
            orientation_ = (orientation_ * dq).normalized();
        }

        const Eigen::Vector3d acc_unbiased = acc - acc_bias_;
        const double gain = options_.acceleration_lpf_tau <= 0.0
                                ? 1.0
                                : std::clamp(
                                      dt / (options_.acceleration_lpf_tau + dt),
                                      0.0,
                                      1.0);
        if (!has_acc_lpf_)
        {
            acc_lpf_ = acc_unbiased;
            has_acc_lpf_ = true;
        }
        else
        {
            acc_lpf_ = gain * acc_unbiased + (1.0 - gain) * acc_lpf_;
        }

        Eigen::Vector3d acc_world = orientation_.toRotationMatrix() * acc_lpf_;
        if (acc_includes_gravity_)
        {
            acc_world += Eigen::Vector3d(0.0, 0.0, -options_.gravity);
        }
        velocity_world_ += acc_world * dt;

        const double speed = velocity_world_.norm();
        if (speed > options_.max_speed)
        {
            velocity_world_ *= options_.max_speed / speed;
        }

        PublishLocked(ResolveStampLocked(stamp));
    }

    void InitializeLocked(double sensor_stamp)
    {
        if (options_.estimate_initial_bias && preinit_count_ >= kMinBiasSamples)
        {
            const double count = static_cast<double>(preinit_count_);
            gyro_bias_ = preinit_gyro_sum_ / count;
            const Eigen::Vector3d mean_acc = preinit_acc_sum_ / count;
            acc_includes_gravity_ =
                std::abs(mean_acc.norm() - options_.gravity) < 3.0;
            if (acc_includes_gravity_)
            {
                const Eigen::Vector3d expected_acc =
                    orientation_.toRotationMatrix().transpose() *
                    Eigen::Vector3d(0.0, 0.0, options_.gravity);
                acc_bias_ = mean_acc - expected_acc;
            }
            else
            {
                acc_bias_ = mean_acc;
            }
        }

        last_imu_stamp_ = has_pair_stamp_ ? latest_pair_stamp_ : sensor_stamp;
        velocity_world_.setZero();
        initialized_ = true;
    }

    rclcpp::Time ResolveStampLocked(double sensor_stamp) const
    {
        if (options_.restamp_to_now && clock_)
        {
            return clock_->now();
        }
        return get_ros_time(sensor_stamp);
    }

    void PublishLocked(const rclcpp::Time &stamp)
    {
        if (!publisher_ || !initialized_)
        {
            return;
        }

        const Eigen::Vector3d velocity_base =
            orientation_.conjugate() * velocity_world_;
        const Eigen::Vector3d angular_velocity =
            latest_gyro_ - gyro_bias_;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = options_.odom_frame;
        odom.child_frame_id = options_.base_frame;
        odom.pose.pose.position.x = position_.x();
        odom.pose.pose.position.y = position_.y();
        odom.pose.pose.position.z = position_.z();
        odom.pose.pose.orientation.x = orientation_.x();
        odom.pose.pose.orientation.y = orientation_.y();
        odom.pose.pose.orientation.z = orientation_.z();
        odom.pose.pose.orientation.w = orientation_.w();
        odom.twist.twist.linear.x = velocity_base.x();
        odom.twist.twist.linear.y = velocity_base.y();
        odom.twist.twist.linear.z = velocity_base.z();
        odom.twist.twist.angular.x = angular_velocity.x();
        odom.twist.twist.angular.y = angular_velocity.y();
        odom.twist.twist.angular.z = angular_velocity.z();
        publisher_->publish(odom);
    }

    static constexpr size_t kMaxQueueSize = 64;
    static constexpr int kMinBiasSamples = 20;
    static constexpr double kMaxImuDt = 0.2;

    std::mutex mutex_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;
    Options options_;
    std::deque<Sample> primary_queue_;
    std::deque<Sample> secondary_queue_;

    bool initialized_ = false;
    bool has_pair_stamp_ = false;
    bool has_acc_lpf_ = false;
    bool acc_includes_gravity_ = true;
    double last_imu_stamp_ = 0.0;
    double latest_pair_stamp_ = 0.0;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity_world_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_lpf_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d preinit_gyro_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d preinit_acc_sum_ = Eigen::Vector3d::Zero();
    int preinit_count_ = 0;
};

class ImuOdomFusion
{
public:
    ImuOdomFusion(const nav_msgs::msg::Path &path_template,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &odom_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &path_pub,
                  const rclcpp::Clock::SharedPtr &clock,
                  tf2_ros::TransformBroadcaster *tf_pub,
                  const std::string &map_frame,
                  const std::string &base_frame)
        : odom_pub_(odom_pub),
          path_pub_(path_pub),
          clock_(clock),
          tf_pub_(tf_pub),
          map_frame_(map_frame),
          base_frame_(base_frame),
          fused_path_(path_template)
    {
        ori_x_.setZero();
        pos_x_.setZero();
        ori_p_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
        pos_p_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;

        ori_q_.setZero();
        ori_q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 3e-3;
        ori_q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 3e-5;
        ori_r_ = Eigen::Matrix3d::Identity() * 4e-4;

        pos_q_ = Eigen::Matrix<double, 6, 6>::Identity() * 5e-3;
        pos_r_ = Eigen::Matrix3d::Identity() * 2e-3;
        pos_r_(2, 2) = 2e-2;
        vel_r_ = Eigen::Matrix3d::Identity() * 0.12;
        vel_r_(2, 2) = 2.0;
    }

    void SetImuToBaseRotation(const Eigen::Matrix3d &r_base_imu)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        r_base_imu_ = r_base_imu;
    }

    void SetAftToBaseTransform(const Eigen::Vector3d &t_aft_base, const Eigen::Vector3d &rpy_aft_base)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        t_aft_to_base_ = t_aft_base;
        q_aft_to_base_ = RpyToQuat(rpy_aft_base);
        q_aft_to_base_.normalize();
        q_base_to_aft_ = q_aft_to_base_.conjugate();

        // IMU is assumed colocated with aft_mapped.
        // Convert imu(aft) gyro to base_link gyro for base_link attitude prediction.
        r_base_imu_ = q_aft_to_base_.toRotationMatrix();
    }

    void SetBaseLinkFrame(const std::string &base_link_frame)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        base_link_frame_ = base_link_frame;
    }

    void SetRestampOutputsToNow(bool enabled)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        restamp_outputs_to_now_ = enabled;
    }

    void SetMaxPathPoses(size_t max_path_poses)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        max_path_poses_ = max_path_poses;
        TrimPathLocked();
    }

    rclcpp::Time OnLioMeasurement(const SE3 &pose, double stamp)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const auto synchronized_output_stamp = ResolveOutputStampLocked(stamp, nullptr);
        const Eigen::Vector3d p_meas = pose.translation();
        position_xyz_ = p_meas;

        Eigen::Quaterniond q_meas_aft(pose.rotationMatrix());
        q_meas_aft.normalize();
        const Eigen::Quaterniond q_meas_base = q_meas_aft * q_aft_to_base_;
        const Eigen::Vector3d rpy_meas = QuatToRpy(q_meas_base);

        if (!initialized_)
        {
            ori_x_.segment<3>(0) = rpy_meas;
            NormalizeRPYInPlace(ori_x_);
            orientation_q_ = RpyToQuat(ori_x_.segment<3>(0));

            if (preinit_imu_count_ > 20)
            {
                ori_x_.segment<3>(3) = preinit_gyro_sum_ / static_cast<double>(preinit_imu_count_);
                const Eigen::Vector3d mean_acc = preinit_acc_sum_ / static_cast<double>(preinit_imu_count_);
                imu_acc_includes_gravity_ = std::abs(mean_acc.norm() - gravity_) < 3.0;
                const Eigen::Vector3d expected_static_acc_base =
                    orientation_q_.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, gravity_);
                if (imu_acc_includes_gravity_)
                {
                    acc_bias_base_ = mean_acc - expected_static_acc_base;
                }
                else
                {
                    acc_bias_base_ = mean_acc;
                }
            }

            initialized_ = true;
            last_imu_t_ = stamp;
            last_meas_t_ = stamp;
            history_.clear();
            AppendPathAndPublishLocked(stamp, &synchronized_output_stamp);
            return synchronized_output_stamp;
        }

        if (history_.empty() || stamp >= history_.back().stamp)
        {
            UpdateOrientationLocked(rpy_meas);
            last_meas_t_ = stamp;
            if (!history_.empty() && std::abs(stamp - history_.back().stamp) < 1e-4)
            {
                SaveSnapshotLocked(history_.back());
            }
            const double publish_stamp = history_.empty() ? stamp : history_.back().stamp;
            AppendPathAndPublishLocked(publish_stamp, &synchronized_output_stamp);
            return synchronized_output_stamp;
        }

        int idx = -1;
        for (int i = static_cast<int>(history_.size()) - 1; i >= 0; --i)
        {
            if (history_[static_cast<size_t>(i)].stamp <= stamp)
            {
                idx = i;
                break;
            }
        }
        if (idx < 0)
        {
            idx = 0;
        }

        LoadSnapshotLocked(history_[static_cast<size_t>(idx)]);
        UpdateOrientationLocked(rpy_meas);
        SaveSnapshotLocked(history_[static_cast<size_t>(idx)]);

        for (size_t j = static_cast<size_t>(idx) + 1; j < history_.size(); ++j)
        {
            PredictAllLocked(history_[j].gyro, history_[j].acc, history_[j].dt);
            last_imu_t_ = history_[j].stamp;
            SaveSnapshotLocked(history_[j]);
        }

        last_meas_t_ = stamp;
        AppendPathAndPublishLocked(history_.back().stamp, &synchronized_output_stamp);
        return synchronized_output_stamp;
    }

    void OnImu(const IMUPtr &imu)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const Vec3d gyro_base = r_base_imu_ * imu->gyro_;
        const Vec3d acc_base = r_base_imu_ * imu->acce_;

        if (!initialized_)
        {
            preinit_gyro_sum_ += gyro_base;
            preinit_acc_sum_ += acc_base;
            preinit_imu_count_++;
            return;
        }

        const double t = imu->timestamp_;
        double dt = t - last_imu_t_;
        if (dt <= 0.0)
        {
            return;
        }

        if (dt > kMaxImuDtSec)
        {
            last_imu_t_ = t;
            history_.clear();
            return;
        }

        PredictAllLocked(gyro_base, acc_base, dt);
        last_imu_t_ = t;

        HistoryEntry e;
        e.stamp = t;
        e.dt = dt;
        e.gyro = gyro_base;
        e.acc = acc_base;
        SaveSnapshotLocked(e);
        history_.push_back(e);

        while (!history_.empty() && (t - history_.front().stamp) > history_sec_)
        {
            history_.pop_front();
        }

        if ((t - last_meas_t_) > max_predict_without_meas_)
        {
            return;
        }

        PublishOdomLocked(t);
    }

private:
    struct HistoryEntry
    {
        double stamp = 0.0;
        double dt = 0.0;
        Vec3d gyro = Vec3d::Zero();
        Vec3d acc = Vec3d::Zero();

        Eigen::Matrix<double, 6, 1> ori_x = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 6> ori_p = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Quaterniond orientation_q = Eigen::Quaterniond::Identity();

        Eigen::Matrix<double, 6, 1> pos_x = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 6> pos_p = Eigen::Matrix<double, 6, 6>::Identity();

        bool has_last_pos_meas = false;
        double last_pos_meas_t = 0.0;
        Eigen::Vector3d last_pos_meas = Eigen::Vector3d::Zero();
    };

    static Eigen::Quaterniond RpyToQuat(const Eigen::Vector3d &rpy)
    {
        tf2::Quaternion q;
        q.setRPY(rpy.x(), rpy.y(), rpy.z());
        return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    static Eigen::Vector3d QuatToRpy(const Eigen::Quaterniond &q)
    {
        tf2::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    static double NormalizeAngle(double a)
    {
        constexpr double kPi = 3.1415926535897932384626433832795;
        a = std::fmod(a + kPi, 2.0 * kPi);
        if (a < 0.0)
        {
            a += 2.0 * kPi;
        }
        return a - kPi;
    }

    static void NormalizeRPYInPlace(Eigen::Matrix<double, 6, 1> &x)
    {
        x(0) = NormalizeAngle(x(0));
        x(1) = NormalizeAngle(x(1));
        x(2) = NormalizeAngle(x(2));
    }

    static Eigen::Vector3d NormalizeInnovation(const Eigen::Vector3d &y)
    {
        return Eigen::Vector3d(
            NormalizeAngle(y.x()),
            NormalizeAngle(y.y()),
            NormalizeAngle(y.z()));
    }

    void SaveSnapshotLocked(HistoryEntry &e)
    {
        e.ori_x = ori_x_;
        e.ori_p = ori_p_;
        e.orientation_q = orientation_q_;

        e.pos_x = pos_x_;
        e.pos_p = pos_p_;
        e.has_last_pos_meas = has_last_pos_meas_;
        e.last_pos_meas_t = last_pos_meas_t_;
        e.last_pos_meas = last_pos_meas_;
    }

    void LoadSnapshotLocked(const HistoryEntry &e)
    {
        ori_x_ = e.ori_x;
        ori_p_ = e.ori_p;
        orientation_q_ = e.orientation_q;

        pos_x_ = e.pos_x;
        pos_p_ = e.pos_p;
        has_last_pos_meas_ = e.has_last_pos_meas;
        last_pos_meas_t_ = e.last_pos_meas_t;
        last_pos_meas_ = e.last_pos_meas;
    }

    void PredictAllLocked(const Vec3d &gyro, const Vec3d &acc, double dt)
    {
        (void)acc;
        PredictOrientationLocked(gyro, dt);
    }

    void PredictOrientationLocked(const Vec3d &gyro, double dt)
    {
        if (!(dt > 0.0))
        {
            return;
        }

        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * dt;

        const Eigen::Vector3d omega = gyro - ori_x_.segment<3>(3);
        Eigen::Quaterniond dq(1.0,
                              0.5 * omega.x() * dt,
                              0.5 * omega.y() * dt,
                              0.5 * omega.z() * dt);
        dq.normalize();
        orientation_q_ = (orientation_q_ * dq).normalized();

        ori_x_.segment<3>(0) = QuatToRpy(orientation_q_);
        NormalizeRPYInPlace(ori_x_);

        const Eigen::Matrix<double, 6, 6> Qd = ori_q_ * dt;
        ori_p_ = F * ori_p_ * F.transpose() + Qd;
    }

    void UpdateOrientationLocked(const Eigen::Vector3d &rpy_meas)
    {
        Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        const Eigen::Vector3d y = NormalizeInnovation(rpy_meas - ori_x_.segment<3>(0));
        const Eigen::Matrix3d S = H * ori_p_ * H.transpose() + ori_r_;
        const Eigen::Matrix<double, 6, 3> K = ori_p_ * H.transpose() * S.ldlt().solve(Eigen::Matrix3d::Identity());

        ori_x_ += K * y;
        NormalizeRPYInPlace(ori_x_);
        orientation_q_ = RpyToQuat(ori_x_.segment<3>(0));

        const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 6> KH = K * H;
        ori_p_ = (I - KH) * ori_p_ * (I - KH).transpose() + K * ori_r_ * K.transpose();
    }

    void PredictPositionLocked(const Vec3d &acc_body, double dt)
    {
        if (!(dt > 0.0))
        {
            return;
        }

        const Eigen::Vector3d acc_unbiased = acc_body - acc_bias_base_;
        const double tau = 0.06;
        const double alpha = std::clamp(dt / (tau + dt), 0.0, 1.0);
        if (!has_acc_lpf_)
        {
            acc_lpf_base_ = acc_unbiased;
            has_acc_lpf_ = true;
        }
        else
        {
            acc_lpf_base_ = alpha * acc_unbiased + (1.0 - alpha) * acc_lpf_base_;
        }

        const Eigen::Matrix3d rot = orientation_q_.toRotationMatrix();
        Eigen::Vector3d acc_world = rot * acc_lpf_base_;
        if (imu_acc_includes_gravity_)
        {
            acc_world += Eigen::Vector3d(0.0, 0.0, -gravity_);
        }

        Eigen::Vector3d pos = pos_x_.segment<3>(0);
        Eigen::Vector3d vel = pos_x_.segment<3>(3);
        pos += vel * dt + 0.5 * acc_world * dt * dt;
        vel += acc_world * dt;
        pos_x_.segment<3>(0) = pos;
        pos_x_.segment<3>(3) = vel;

        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;
        const double sigma_a = acc_noise_std_ + 0.2 * acc_unbiased.norm();
        const double q = sigma_a * sigma_a;
        Eigen::Matrix<double, 6, 6> q_cv = Eigen::Matrix<double, 6, 6>::Zero();
        q_cv.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.25 * q * dt4);
        q_cv.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (0.5 * q * dt3);
        q_cv.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (0.5 * q * dt3);
        q_cv.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (q * dt2);

        pos_p_ = F * pos_p_ * F.transpose() + q_cv + pos_q_ * dt;
    }

    void UpdatePositionLocked(const Eigen::Vector3d &p_meas, double stamp)
    {
        Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        const Eigen::Vector3d y = p_meas - pos_x_.segment<3>(0);
        const Eigen::Matrix3d S = H * pos_p_ * H.transpose() + pos_r_;
        const Eigen::Matrix<double, 6, 3> K = pos_p_ * H.transpose() * S.ldlt().solve(Eigen::Matrix3d::Identity());

        pos_x_ += K * y;

        const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 6> KH = K * H;
        pos_p_ = (I - KH) * pos_p_ * (I - KH).transpose() + K * pos_r_ * K.transpose();

        if (has_last_pos_meas_)
        {
            const double dt = stamp - last_pos_meas_t_;
            if (dt > 0.03 && dt < 0.4)
            {
                Eigen::Vector3d vel_meas = (p_meas - last_pos_meas_) / dt;
                const double speed = vel_meas.norm();
                const double vmax = 35.0;
                if (speed > vmax)
                {
                    vel_meas *= vmax / speed;
                }

                Eigen::Matrix<double, 3, 6> Hv = Eigen::Matrix<double, 3, 6>::Zero();
                Hv.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

                Eigen::Matrix3d rv = vel_r_;
                rv.diagonal().array() += std::min(2.0, 0.08 * speed);

                const Eigen::Vector3d yv = vel_meas - pos_x_.segment<3>(3);
                const Eigen::Matrix3d Sv = Hv * pos_p_ * Hv.transpose() + rv;
                const Eigen::Matrix<double, 6, 3> Kv =
                    pos_p_ * Hv.transpose() * Sv.ldlt().solve(Eigen::Matrix3d::Identity());

                pos_x_ += Kv * yv;
                const Eigen::Matrix<double, 6, 6> KHv = Kv * Hv;
                pos_p_ = (I - KHv) * pos_p_ * (I - KHv).transpose() + Kv * rv * Kv.transpose();
            }
        }

        has_last_pos_meas_ = true;
        last_pos_meas_t_ = stamp;
        last_pos_meas_ = p_meas;
    }

    rclcpp::Time ResolveOutputStampLocked(
        double sensor_stamp, const rclcpp::Time *synchronized_output_stamp) const
    {
        if (!restamp_outputs_to_now_)
        {
            return get_ros_time(sensor_stamp);
        }
        if (synchronized_output_stamp)
        {
            return *synchronized_output_stamp;
        }
        return clock_ ? clock_->now() : get_ros_time(sensor_stamp);
    }

    void PublishOdomLocked(
        double sensor_stamp, const rclcpp::Time *synchronized_output_stamp = nullptr)
    {
        const Eigen::Vector3d pos = position_xyz_;
        const Eigen::Quaterniond q = orientation_q_ * q_base_to_aft_;
        const auto output_stamp =
            ResolveOutputStampLocked(sensor_stamp, synchronized_output_stamp);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = output_stamp;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = pos.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom_pub_->publish(odom);

        if (tf_pub_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = pos.x();
            tf.transform.translation.y = pos.y();
            tf.transform.translation.z = pos.z();
            tf.transform.rotation = odom.pose.pose.orientation;

            geometry_msgs::msg::TransformStamped tf_base_link;
            tf_base_link.header.stamp = output_stamp;
            tf_base_link.header.frame_id = base_frame_;
            tf_base_link.child_frame_id = base_link_frame_;
            tf_base_link.transform.translation.x = t_aft_to_base_.x();
            tf_base_link.transform.translation.y = t_aft_to_base_.y();
            tf_base_link.transform.translation.z = t_aft_to_base_.z();
            tf_base_link.transform.rotation.x = q_aft_to_base_.x();
            tf_base_link.transform.rotation.y = q_aft_to_base_.y();
            tf_base_link.transform.rotation.z = q_aft_to_base_.z();
            tf_base_link.transform.rotation.w = q_aft_to_base_.w();

            tf_pub_->sendTransform(tf);
            tf_pub_->sendTransform(tf_base_link);
        }
    }

    void AppendPathAndPublishLocked(
        double sensor_stamp, const rclcpp::Time *synchronized_output_stamp = nullptr)
    {
        const auto output_stamp =
            ResolveOutputStampLocked(sensor_stamp, synchronized_output_stamp);
        PublishOdomLocked(sensor_stamp, &output_stamp);
        if (!path_pub_ || max_path_poses_ == 0 || path_pub_->get_subscription_count() == 0)
        {
            return;
        }
        const Eigen::Quaterniond q = orientation_q_ * q_base_to_aft_;

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = output_stamp;
        ps.header.frame_id = map_frame_;
        ps.pose.position.x = position_xyz_.x();
        ps.pose.position.y = position_xyz_.y();
        ps.pose.position.z = position_xyz_.z();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();

        fused_path_.header = ps.header;
        fused_path_.poses.push_back(ps);
        TrimPathLocked();
        if (path_pub_)
        {
            path_pub_->publish(fused_path_);
        }
    }

    std::mutex mtx_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Clock::SharedPtr clock_;
    tf2_ros::TransformBroadcaster *tf_pub_ = nullptr;
    std::string map_frame_;
    std::string base_frame_;
    std::string base_link_frame_ = "base_link";
    bool restamp_outputs_to_now_ = true;
    size_t max_path_poses_ = 2000;

    bool initialized_ = false;
    double last_imu_t_ = 0.0;
    double last_meas_t_ = 0.0;
    double history_sec_ = 2.0;
    double max_predict_without_meas_ = 1.0;
    double gravity_ = 9.81;
    double acc_noise_std_ = 0.20;
    Eigen::Matrix3d r_base_imu_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d acc_bias_base_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_lpf_base_ = Eigen::Vector3d::Zero();
    bool has_acc_lpf_ = false;
    bool has_last_pos_meas_ = false;
    double last_pos_meas_t_ = 0.0;
    Eigen::Vector3d last_pos_meas_ = Eigen::Vector3d::Zero();
    bool imu_acc_includes_gravity_ = true;
    Eigen::Vector3d preinit_acc_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d preinit_gyro_sum_ = Eigen::Vector3d::Zero();
    int preinit_imu_count_ = 0;

    static constexpr double kMaxImuDtSec = 0.2;

    Eigen::Matrix<double, 6, 1> ori_x_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> ori_p_ = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> ori_q_ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d ori_r_ = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond orientation_q_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_base_to_aft_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_aft_to_base_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_aft_to_base_ = Eigen::Quaterniond::Identity();

    Eigen::Vector3d position_xyz_ = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 1> pos_x_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> pos_p_ = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> pos_q_ = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix3d pos_r_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d vel_r_ = Eigen::Matrix3d::Identity();
    std::deque<HistoryEntry> history_;
    nav_msgs::msg::Path fused_path_;

    void TrimPathLocked()
    {
        if (max_path_poses_ == 0 || fused_path_.poses.size() <= max_path_poses_)
        {
            return;
        }

        fused_path_.poses.erase(
            fused_path_.poses.begin(),
            fused_path_.poses.begin() + (fused_path_.poses.size() - max_path_poses_));
    }
};

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
    // cloud_vec	一整帧 Livox 点云，被切成的多个子点云
    // cloud_out	第 i 个子点云（时间片）
    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;

    // Use the ROS shared pointer directly to avoid copying the full Livox message.
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out, false); },
                                   "laser convert");

    if (cloud_vec.empty() || t_out.empty())
    {
        return;
    }

    const size_t segment_count = std::min(cloud_vec.size(), t_out.size());
    for (size_t i = 0; i < segment_count; i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
        // double sample_size = 0.01;
        thread_local std::mt19937_64 g;
        zjloc::common::Timer::Evaluate([&]()
                                       { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            sub_sample_frame(cloud_out, sample_size); },
                                       "laser ds");
        lio->pushData(std::move(cloud_out), std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]), false);
        // pair<本段数据的绝对起始时间,数据持续时长>
    }
}

void aux_livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
    std::vector<std::vector<point3D>> cloud_vec; // 附属雷达只存于第0个时间片
    std::vector<double> t_out;                   // 只有第0个时间片，代表总体的时间长度

    // Keep auxiliary LiDAR behavior unchanged; only remove the full-message copy.
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out, true); },
                                   "laser convert");

    if (cloud_vec.empty() || t_out.empty())
    {
        return;
    }

    auto &cloud_out = cloud_vec[0];
    double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
    // double sample_size = 0.01;
    thread_local std::mt19937_64 g;
    zjloc::common::Timer::Evaluate([&]()
                                   { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        sub_sample_frame(cloud_out, sample_size);
        // Auxiliary soft synchronization uses a monotonic point cursor.
        // sub_sample_frame() is hash-map based, so restore timestamp order.
        std::sort(cloud_out.begin(), cloud_out.end(), [](const point3D &a, const point3D &b) {
            return a.timestamp < b.timestamp;
        }); },
                                   "laser ds");

    lio->pushData(std::move(cloud_out), std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, t_out[0]), true);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // sensor_msgs::msg::PointCloud2::SharedPtr cloud(new sensor_msgs::msg::PointCloud2(*msg));

    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out); },
                                   "laser convert");

    const size_t segment_count = std::min(cloud_vec.size(), t_out.size());
    for (size_t i = 0; i < segment_count; i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;
        // double sample_size = 0.05;
        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            thread_local std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            sub_sample_frame(cloud_out, sample_size);
        },
                                       "laser ds");

        // 在ROS2中使用的是nanoseconds().count()来获取时间戳
        lio->pushData(std::move(cloud_out), std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]), false);
    }
}

void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    // sensor_msgs::msg::Imu::SharedPtr msg_temp(new sensor_msgs::msg::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * gnorm);
    lio->pushData(imu);
    if (imu_odom_fusion)
    {
        imu_odom_fusion->OnImu(imu);
    }
    if (dual_imu_center_odom)
    {
        // Side-channel only: the main LIO and its single-IMU fusion have already
        // consumed the primary IMU above.
        dual_imu_center_odom->AddPrimary(imu);
    }
    // {
    //     msg_temp->linear_acceleration.x = msg_temp->linear_acceleration.x * gnorm;
    //     msg_temp->linear_acceleration.y = msg_temp->linear_acceleration.y * gnorm;
    //     msg_temp->linear_acceleration.z = msg_temp->linear_acceleration.z * gnorm;
    //     imu_repub->publish(*msg_temp);
    // }
}

void auxImuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!dual_imu_center_odom)
    {
        return;
    }

    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z) *
            gnorm);
    dual_imu_center_odom->AddSecondary(imu);
}

int main(int argc, char **argv)
{
    // 处理命令行参数，将gflags参数与ROS2参数分离
    // Debug: print initial argv for diagnosing flag parsing issues
    std::cout << "[adaptive_lio] argv before init:";
    for (int i = 0; i < argc; ++i)
    {
        std::cout << " '" << (argv[i] ? argv[i] : std::string("(null)")) << "'";
    }
    std::cout << std::endl;

    // Initialize logging and ROS2. Important: call rclcpp::init before
    // parsing Google gflags so ROS2's --ros-args (and -r) are handled by
    // rclcpp and not consumed (or rejected) by gflags.
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    rclcpp::init(argc, argv);

    // Build a filtered argv for gflags so ROS2 launch args are not seen by gflags.
    // Keep program name.
    std::vector<char *> gflags_argv;
    gflags_argv.push_back(argv[0]);
    for (int i = 1; i < argc; ++i)
    {
        std::string a(argv[i]);
        // Skip ROS2-specific markers and remappings
        if (a == "--ros-args")
        {
            continue;
        }
        if (a.rfind("--ros", 0) == 0)
        {
            continue;
        }
        if (a == "-r")
        {
            // remap flag, skip it (its argument is the remapping spec which
            // usually starts with __node:= or topic remap; skip that token too)
            continue;
        }
        if (a.rfind("__", 0) == 0)
        {
            // __node:=... or other remap specs
            continue;
        }
        if (a == "--params-file")
        {
            // skip params-file and its value (if present)
            ++i;
            continue;
        }
        // otherwise keep the arg for gflags
        gflags_argv.push_back(argv[i]);
    }

    int gflags_argc = static_cast<int>(gflags_argv.size());
    char **gflags_argv_ptr = gflags_argv.data();
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);
    auto node = rclcpp::Node::make_shared("adaptive_lio_node");

    std::string config_file = FLAGS_config_file.empty()
                                  ? std::string(ROOT_DIR) + "config/mapping_m.yaml"
                                  : FLAGS_config_file;
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    // Init Scantext Modules
    scantext_mapping = std::make_shared<scantext::MappingCore>();
    scantext::SCParams scantext_params;

    // Load configs
    try
    {
        auto yaml_mapping_ = YAML::LoadFile(config_file);
        if (yaml_mapping_["mapping_module"])
        {
            scantext::MappingCore::Config cfg;
            auto node = yaml_mapping_["mapping_module"];
            if (node["enable_mapping"])
                cfg.enable_mapping = node["enable_mapping"].as<bool>();
            if (node["keyframe_dist_thresh"])
                cfg.keyframe_dist_thresh = node["keyframe_dist_thresh"].as<double>();
            if (node["keyframe_angle_thresh"])
                cfg.keyframe_angle_thresh = node["keyframe_angle_thresh"].as<double>();
            if (node["map_save_path"])
                cfg.map_save_path = node["map_save_path"].as<std::string>();
            if (node["auto_save_interval"])
                cfg.auto_save_interval = node["auto_save_interval"].as<double>();
            scantext_mapping->setConfig(cfg);

            if (node["scantext"])
            {
                auto sc_node = node["scantext"];
                if (sc_node["num_ring"])
                    scantext_params.num_ring = sc_node["num_ring"].as<int>();
                if (sc_node["num_sector"])
                    scantext_params.num_sector = sc_node["num_sector"].as<int>();
                if (sc_node["max_radius"])
                    scantext_params.max_radius = sc_node["max_radius"].as<double>();
                if (sc_node["lidar_height"])
                    scantext_params.lidar_height = sc_node["lidar_height"].as<double>();
                if (sc_node["use_scpp"])
                    scantext_params.use_scpp = sc_node["use_scpp"].as<bool>();
                if (sc_node["scpp_search_ratio"])
                    scantext_params.scpp_search_ratio = sc_node["scpp_search_ratio"].as<double>();
                if (sc_node["cart_x_unit"])
                    scantext_params.cart_x_unit = sc_node["cart_x_unit"].as<double>();
                if (sc_node["cart_y_unit"])
                    scantext_params.cart_y_unit = sc_node["cart_y_unit"].as<double>();
                if (sc_node["cart_x_max"])
                    scantext_params.cart_x_max = sc_node["cart_x_max"].as<double>();
                if (sc_node["cart_y_max"])
                    scantext_params.cart_y_max = sc_node["cart_y_max"].as<double>();
            }
            scantext_mapping->setScanContextParams(scantext_params);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading scantext configs: " << e.what() << std::endl;
    }

    lio = new zjloc::lidarodom_m();
    if (!lio->init(config_file))
    {
        return -1;
    }

    auto yaml_cfg = YAML::LoadFile(config_file);
    bool restamp_outputs_to_now = true;
    if (yaml_cfg["common"])
    {
        const auto common = yaml_cfg["common"];
        if (common["fusion_restamp_outputs_to_now"])
        {
            restamp_outputs_to_now = common["fusion_restamp_outputs_to_now"].as<bool>();
        }
        else if (common["fusion_restamp_tf_to_now"])
        {
            // Backward compatibility with the old TF-only setting.
            restamp_outputs_to_now = common["fusion_restamp_tf_to_now"].as<bool>();
        }
    }
    FrameStampSynchronizer frame_stamp_synchronizer;

    auto pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/scan", rclcpp::SensorDataQoS().keep_last(1));

    struct ScanPublishTask
    {
        zjloc::CloudPtr cloud;
        double sensor_stamp;
    };

    std::deque<ScanPublishTask> scan_publish_queue;
    std::mutex scan_publish_queue_mutex;
    std::condition_variable scan_publish_queue_cv;
    bool stop_scan_publish_thread = false;
    std::thread scan_publish_worker([&]()
                                    {
        while (true)
        {
            ScanPublishTask task;
            {
                std::unique_lock<std::mutex> lock(scan_publish_queue_mutex);
                scan_publish_queue_cv.wait(lock, [&]() {
                    return stop_scan_publish_thread || !scan_publish_queue.empty();
                });

                if (stop_scan_publish_thread && scan_publish_queue.empty())
                {
                    break;
                }

                task = std::move(scan_publish_queue.front());
                scan_publish_queue.pop_front();
            }

            if (!task.cloud)
            {
                continue;
            }

            rclcpp::Time output_stamp(0, 0);
            if (!frame_stamp_synchronizer.WaitLookup(
                    task.sensor_stamp, output_stamp, std::chrono::milliseconds(2000)))
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(), *node->get_clock(), 2000,
                    "Dropping /livox/scan: no synchronized TF stamp for sensor time %.9f",
                    task.sensor_stamp);
                continue;
            }

            sensor_msgs::msg::PointCloud2 cloud_output;
            pcl::toROSMsg(*task.cloud, cloud_output);
            cloud_output.header.stamp = output_stamp;
            cloud_output.header.frame_id = "odom";
            pub_scan->publish(cloud_output);
        } });

    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            if (topic_name != "laser" || pub_scan->get_subscription_count() == 0)
            {
                return true;
            }

            {
                std::lock_guard<std::mutex> lock(scan_publish_queue_mutex);
                if (scan_publish_queue.size() >= 2)
                {
                    scan_publish_queue.pop_front();
                }
                scan_publish_queue.push_back({cloud, time});
            }
            scan_publish_queue_cv.notify_one();
            return true;
        }

    );

    auto pubLaserOdometry = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
    auto pubLaserOdometryPath = node->create_publisher<nav_msgs::msg::Path>("/odometry_path", 5);

    bool center_odom_enabled = false;
    std::string aux_imu_topic;
    std::string center_odom_topic = "/odom_base_link";
    if (yaml_cfg["common"])
    {
        const auto common = yaml_cfg["common"];
        center_odom_enabled =
            common["center_odom_enable"] && common["center_odom_enable"].as<bool>();
        if (common["aux_imu_topic"])
        {
            aux_imu_topic = common["aux_imu_topic"].as<std::string>();
        }
        if (common["center_odom_topic"])
        {
            center_odom_topic = common["center_odom_topic"].as<std::string>();
        }
    }

    if (center_odom_enabled && aux_imu_topic.empty())
    {
        RCLCPP_WARN(
            node->get_logger(),
            "center_odom_enable is true but aux_imu_topic is empty; "
            "disabling the dual-IMU side channel");
        center_odom_enabled = false;
    }

    if (center_odom_enabled)
    {
        DualImuCenterOdom::Options options;
        options.restamp_to_now = restamp_outputs_to_now;
        const auto common = yaml_cfg["common"];
        if (common["fusion_base_link_frame"])
        {
            options.base_frame = common["fusion_base_link_frame"].as<std::string>();
        }
        if (common["fusion_aft_to_base_xyzrpy"])
        {
            const auto values =
                common["fusion_aft_to_base_xyzrpy"].as<std::vector<double>>();
            if (values.size() == 6)
            {
                options.sensor_to_base_translation =
                    Eigen::Vector3d(values[0], values[1], values[2]);
                options.sensor_to_base_rotation =
                    quaternion_from_rpy(Eigen::Vector3d(values[3], values[4], values[5]));
            }
        }
        const auto read_rotation = [&](const char *key) -> Eigen::Matrix3d {
            if (!common[key])
            {
                return Eigen::Matrix3d::Identity();
            }
            const auto values = common[key].as<std::vector<double>>();
            if (values.size() != 3)
            {
                RCLCPP_WARN(
                    node->get_logger(),
                    "%s must contain [roll, pitch, yaw]; using identity",
                    key);
                return Eigen::Matrix3d::Identity();
            }
            return quaternion_from_rpy(
                       Eigen::Vector3d(values[0], values[1], values[2]))
                .toRotationMatrix();
        };
        options.primary_imu_to_base_rotation =
            read_rotation("center_primary_imu_to_base_rpy");
        options.secondary_imu_to_base_rotation =
            read_rotation("center_secondary_imu_to_base_rpy");
        if (common["center_secondary_imu_time_offset"])
        {
            options.secondary_time_offset =
                common["center_secondary_imu_time_offset"].as<double>();
        }
        if (common["center_imu_sync_tolerance"])
        {
            options.sync_tolerance =
                common["center_imu_sync_tolerance"].as<double>();
        }
        if (common["center_acceleration_lpf_tau"])
        {
            options.acceleration_lpf_tau =
                common["center_acceleration_lpf_tau"].as<double>();
        }
        if (common["center_max_speed"])
        {
            options.max_speed = common["center_max_speed"].as<double>();
        }
        if (common["center_gravity"])
        {
            options.gravity = common["center_gravity"].as<double>();
        }
        if (common["center_estimate_initial_bias"])
        {
            options.estimate_initial_bias =
                common["center_estimate_initial_bias"].as<bool>();
        }

        auto pubCenterOdometry =
            node->create_publisher<nav_msgs::msg::Odometry>(center_odom_topic, 100);
        dual_imu_center_odom = std::make_shared<DualImuCenterOdom>(
            pubCenterOdometry, node->get_clock(), options);
    }
    else
    {
        dual_imu_center_odom.reset();
    }

    // 创建tf广播器
    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    imu_odom_fusion = std::make_shared<ImuOdomFusion>(
        laserOdoPath,
        pubLaserOdometry,
        pubLaserOdometryPath,
        node->get_clock(),
        g_tf_broadcaster.get(),
        "odom",
        "aft_mapped");

    // User setup: IMU is colocated/aligned with aft_mapped frame
    imu_odom_fusion->SetImuToBaseRotation(Eigen::Matrix3d::Identity());
    imu_odom_fusion->SetAftToBaseTransform(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    imu_odom_fusion->SetBaseLinkFrame("base_link");

    if (yaml_cfg["common"])
    {
        const auto common = yaml_cfg["common"];
        if (common["fusion_base_link_frame"])
        {
            imu_odom_fusion->SetBaseLinkFrame(common["fusion_base_link_frame"].as<std::string>());
        }
        if (common["fusion_aft_to_base_xyzrpy"])
        {
            auto arr = common["fusion_aft_to_base_xyzrpy"].as<std::vector<double>>();
            if (arr.size() == 6)
            {
                imu_odom_fusion->SetAftToBaseTransform(
                    Eigen::Vector3d(arr[0], arr[1], arr[2]),
                    Eigen::Vector3d(arr[3], arr[4], arr[5]));
            }
        }
        imu_odom_fusion->SetRestampOutputsToNow(restamp_outputs_to_now);
        if (common["fusion_max_path_poses"])
        {
            imu_odom_fusion->SetMaxPathPoses(common["fusion_max_path_poses"].as<size_t>());
        }
    }

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            if (topic_name == "laser")
            {
                if (imu_odom_fusion)
                {
                    const auto output_stamp = imu_odom_fusion->OnLioMeasurement(pose, stamp);
                    // Publish the cloud only after the matching odom/TF has been sent.
                    frame_stamp_synchronizer.Record(stamp, output_stamp);
                    if (dual_imu_center_odom)
                    {
                        dual_imu_center_odom->OnLioPose(pose, stamp, output_stamp);
                    }
                }
            }
            // else if (topic_name == "world")
            // {
            //     transform.header.frame_id = "world";
            //     transform.child_frame_id = "map";
            //     br->sendTransform(transform);
            // }

            return true;
        }

    );

    // Scantext processing queue
    struct ScantextTask
    {
        scantext::ScanContext::SCDescriptor sc;
        scantext::ScanContext::RingKey rk;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ds;
        Eigen::Isometry3d odom_pose;
        double time;
    };

    const bool scantext_enabled = scantext_mapping && scantext_mapping->getConfig().enable_mapping;

    std::queue<ScantextTask> scantext_queue;
    std::mutex scantext_queue_mutex;
    std::condition_variable scantext_queue_cv;
    bool stop_scantext_thread = false;
    std::thread scantext_worker;

    // Consumer thread function
    if (scantext_enabled)
    {
        scantext_worker = std::thread([&]()
                                      {
    while (!stop_scantext_thread) {
        ScantextTask task;
        {
            std::unique_lock<std::mutex> lock(scantext_queue_mutex);
            // Wait for task or stop signal
            scantext_queue_cv.wait(lock, [&]() { return !scantext_queue.empty() || stop_scantext_thread; });

            if (stop_scantext_thread && scantext_queue.empty()) break;

            task = scantext_queue.front();
            scantext_queue.pop();
        }

        if (!scantext_mapping) continue;
        

        // 1) mapping：如果 enable_mapping，且满足内部 keyframe 条件，则入库
        std::shared_ptr<scantext::KeyFrame> new_kf;
        if (scantext_mapping->getConfig().enable_mapping) {
            bool added = scantext_mapping->addFrameWithSC(
                task.cloud_ds, task.odom_pose, task.time, task.sc, task.rk, &new_kf);

            // if (added && new_kf) {
            //     // 增量更新 relo 的 map（重要！否则 mapping 模式下 relo 永远空 map）
            //     scantext_relo->addKeyFrame(new_kf);
            // }
        }
    } });
    }

    Eigen::Isometry3d last_sc_pose = Eigen::Isometry3d::Identity();
    double last_sc_time = 0.0;
    bool has_last_sc = false;

    scantext::ScanContext sc_extractor(scantext_params);

    auto scantext_cbk =
        std::function<bool(const zjloc::CloudPtr &, const SE3 &, double)>(
            [&](const zjloc::CloudPtr &points_world,
                const SE3 &pose,
                double time) -> bool
            {
                if (!scantext_mapping || !points_world || !scantext_mapping->getConfig().enable_mapping)
                    return true;

                // --- pose -> Eigen ---
                Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
                eigen_pose.linear() = pose.rotationMatrix();
                eigen_pose.translation() = pose.translation();

                // --- Step2: 只在关键帧/低频做 SC ---
                const double dist_th =
                    scantext_mapping->getConfig().keyframe_dist_thresh > 1e-6 ? scantext_mapping->getConfig().keyframe_dist_thresh : 1.0;

                const double ang_th =
                    scantext_mapping->getConfig().keyframe_angle_thresh > 1e-6 ? scantext_mapping->getConfig().keyframe_angle_thresh : 0.2;

                const double time_th = 1.0; // 1Hz 兜底（可放到 yaml）

                bool do_sc = false;
                if (!has_last_sc)
                {
                    do_sc = true;
                }
                else
                {
                    Eigen::Isometry3d delta = last_sc_pose.inverse() * eigen_pose;
                    double dist = delta.translation().norm();
                    double ang = Eigen::AngleAxisd(delta.rotation()).angle();
                    double dt = time - last_sc_time;

                    do_sc = (dist >= dist_th) || (ang >= ang_th) || (dt >= time_th);
                }

                if (!do_sc)
                {
                    return true; // 不做 SC，直接返回，不占用队列
                }

                has_last_sc = true;
                last_sc_pose = eigen_pose;
                last_sc_time = time;

                auto cloud_local = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                if (!points_world->empty())
                {
                    const Eigen::Matrix4f t_lidar_world = eigen_pose.matrix().cast<float>().inverse();
                    pcl::transformPointCloud(*points_world, *cloud_local, t_lidar_world);
                }

                // --- Step1: 用当前帧雷达坐标系点云计算 descriptor/ringkey ---
                auto sc = sc_extractor.makeScanContext(*cloud_local);
                auto rk = sc_extractor.makeRingKey(sc);

                // --- 为 ICP / (可选)建图存储 构造一个小云（上限 2000 点）---
                constexpr size_t MAX_DS_PTS = 2000;
                auto cloud_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                if (!cloud_local->empty())
                {
                    cloud_ds->reserve(std::min(cloud_local->size(), MAX_DS_PTS));
                    size_t step = std::max<size_t>(1, cloud_local->size() / MAX_DS_PTS);

                    for (size_t i = 0; i < cloud_local->size(); i += step)
                    {
                        cloud_ds->push_back(cloud_local->points[i]);
                    }
                }

                // --- push 轻量任务到队列 ---
                {
                    std::lock_guard<std::mutex> lock(scantext_queue_mutex);
                    if (scantext_queue.size() < 5)
                    {
                        scantext_queue.push({sc, rk, cloud_ds, eigen_pose, time});
                    }
                }
                scantext_queue_cv.notify_one();

                return true;
            });

    auto vel_pub = node->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
    auto dist_pub = node->create_publisher<std_msgs::msg::Float32>("/move_dist", 1);

    auto save_map_srv =
        node->create_service<std_srvs::srv::Trigger>(
            "save_map_service",
            [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
            {
                std::string path = scantext_mapping->getConfig().map_save_path;
                std::string db_path = std::string(ROOT_DIR) + "map_db";

                scantext_mapping->saveMapAsync(path);

                res->success = true;
                res->message = "Map saving started asynchronously";
            });

    // imu_repub = node->create_publisher<sensor_msgs::msg::Imu>("/repub_imu", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::msg::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub->publish(time_rviz);
            else
                dist_pub->publish(time_rviz);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);
    if (scantext_enabled)
    {
        lio->setFunc(scantext_cbk);
    }

    convert = new zjloc::CloudConvert2;
    convert->LoadFromYAML(config_file);

    lio->setCloudConvert(convert);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    std::string laser_topic = yaml_cfg["common"]["lid_topic"].as<std::string>();
    std::string aux_laser_topic = yaml_cfg["common"]["aux_lidar_topic"].as<std::string>();
    std::string imu_topic = yaml_cfg["common"]["imu_topic"].as<std::string>();
    gnorm = yaml_cfg["common"]["gnorm"].as<double>();

    auto sensor_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sensor_sub_options;
    sensor_sub_options.callback_group = sensor_callback_group;

    // 创建订阅者
    std::shared_ptr<void> subLaserCloud =
        (convert->lidar_type_ == zjloc::CloudConvert2::LidarType::AVIA)
            ? std::static_pointer_cast<void>(node->create_subscription<livox_ros_driver2::msg::CustomMsg>(laser_topic, 100, livox_pcl_cbk, sensor_sub_options))
            : std::static_pointer_cast<void>(node->create_subscription<sensor_msgs::msg::PointCloud2>(laser_topic, 100, standard_pcl_cbk, sensor_sub_options));

    auto subAuxLaserCloud = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(aux_laser_topic, 100, aux_livox_pcl_cbk, sensor_sub_options);

    auto sub_imu_ori = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 500, imuHandler, sensor_sub_options);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_aux_imu;
    if (center_odom_enabled)
    {
        sub_aux_imu = node->create_subscription<sensor_msgs::msg::Imu>(
            aux_imu_topic, 500, auxImuHandler, sensor_sub_options);
        RCLCPP_INFO(
            node->get_logger(),
            "Dual-IMU side channel enabled: %s + %s -> %s; primary LIO remains single-IMU",
            imu_topic.c_str(),
            aux_imu_topic.c_str(),
            center_odom_topic.c_str());
    }

    std::thread measurement_process(&zjloc::lidarodom_m::run, lio);

    const size_t executor_threads = std::max<size_t>(2, std::min<size_t>(4, std::thread::hardware_concurrency()));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), executor_threads);
    executor.add_node(node);
    executor.spin();

    // Cleanup Scantext thread
    {
        std::lock_guard<std::mutex> lock(scantext_queue_mutex);
        stop_scantext_thread = true;
    }
    scantext_queue_cv.notify_all();
    if (scantext_worker.joinable())
        scantext_worker.join();

    {
        std::lock_guard<std::mutex> lock(scan_publish_queue_mutex);
        stop_scan_publish_thread = true;
    }
    scan_publish_queue_cv.notify_all();
    if (scan_publish_worker.joinable())
        scan_publish_worker.join();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}
