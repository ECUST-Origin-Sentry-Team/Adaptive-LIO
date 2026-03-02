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

// ros2 lib
#include "rclcpp/rclcpp.hpp"
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

zjloc::lidarodom_m *lio;
zjloc::CloudConvert2 *convert;
std::shared_ptr<scantext::MappingCore> scantext_mapping;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;
std::shared_ptr<class ImuOdomFusion> imu_odom_fusion;
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

class ImuOdomFusion
{
public:
    ImuOdomFusion(const nav_msgs::msg::Path &path_template,
                  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &odom_pub,
                  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &path_pub,
                  tf2_ros::TransformBroadcaster *tf_pub,
                  const std::string &map_frame,
                  const std::string &base_frame)
        : odom_pub_(odom_pub),
          path_pub_(path_pub),
          tf_pub_(tf_pub),
          map_frame_(map_frame),
          base_frame_(base_frame),
          fused_path_(path_template)
    {
        x_.setZero();
        P_ = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
        Q_.setZero();
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-4;
        R_ = Eigen::Matrix3d::Identity() * 1e-3;
    }

    void OnLioMeasurement(const SE3 &pose, double stamp)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const Eigen::Vector3d p_meas = pose.translation();
        position_xyz_ = p_meas;

        Eigen::Quaterniond q_meas(pose.rotationMatrix());
        q_meas.normalize();
        tf2::Quaternion q_tf(q_meas.x(), q_meas.y(), q_meas.z(), q_meas.w());
        double r = 0.0;
        double p = 0.0;
        double y = 0.0;
        tf2::Matrix3x3(q_tf).getRPY(r, p, y);
        const Eigen::Vector3d rpy_meas(r, p, y);

        if (!initialized_)
        {
            x_.segment<3>(0) = rpy_meas;
            NormalizeRPYInPlace(x_);
            initialized_ = true;
            last_imu_t_ = stamp;
            last_meas_t_ = stamp;
            history_.clear();
            AppendPathAndPublishLocked(stamp);
            return;
        }

        if (history_.empty() || stamp >= history_.back().stamp)
        {
            UpdateRPYLocked(rpy_meas);
            last_meas_t_ = stamp;
            if (!history_.empty() && std::abs(stamp - history_.back().stamp) < 1e-4)
            {
                history_.back().kf_x = x_;
                history_.back().kf_P = P_;
            }
            const double publish_stamp = history_.empty() ? stamp : history_.back().stamp;
            AppendPathAndPublishLocked(publish_stamp);
            return;
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
            return;
        }

        x_ = history_[static_cast<size_t>(idx)].kf_x;
        P_ = history_[static_cast<size_t>(idx)].kf_P;
        UpdateRPYLocked(rpy_meas);
        history_[static_cast<size_t>(idx)].kf_x = x_;
        history_[static_cast<size_t>(idx)].kf_P = P_;

        for (size_t j = static_cast<size_t>(idx) + 1; j < history_.size(); ++j)
        {
            PredictRPYLocked(history_[j].gyro, history_[j].dt);
            last_imu_t_ = history_[j].stamp;
            history_[j].kf_x = x_;
            history_[j].kf_P = P_;
        }

        last_meas_t_ = stamp;
        AppendPathAndPublishLocked(history_.back().stamp);
    }

    void OnImu(const IMUPtr &imu)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!initialized_)
        {
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

        PredictRPYLocked(imu->gyro_, dt);
        last_imu_t_ = t;

        HistoryEntry e;
        e.stamp = t;
        e.dt = dt;
        e.gyro = imu->gyro_;
        e.kf_x = x_;
        e.kf_P = P_;
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
        Eigen::Matrix<double, 6, 1> kf_x = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 6> kf_P = Eigen::Matrix<double, 6, 6>::Identity();
    };

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

    void PredictRPYLocked(const Vec3d &gyro, double dt)
    {
        if (!(dt > 0.0))
        {
            return;
        }

        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * dt;

        x_.segment<3>(0) += (gyro - x_.segment<3>(3)) * dt;
        NormalizeRPYInPlace(x_);

        const Eigen::Matrix<double, 6, 6> Qd = Q_ * dt;
        P_ = F * P_ * F.transpose() + Qd;
    }

    void UpdateRPYLocked(const Eigen::Vector3d &rpy_meas)
    {
        Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        const Eigen::Vector3d y = NormalizeInnovation(rpy_meas - x_.segment<3>(0));
        const Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
        const Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix3d::Identity());

        x_ += K * y;
        NormalizeRPYInPlace(x_);

        const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 6> KH = K * H;
        P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_ * K.transpose();
    }

    void PublishOdomLocked(double stamp)
    {
        tf2::Quaternion q;
        q.setRPY(x_(0), x_(1), x_(2));

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = get_ros_time(stamp);
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = position_xyz_.x();
        odom.pose.pose.position.y = position_xyz_.y();
        odom.pose.pose.position.z = position_xyz_.z();
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
            tf.transform.translation.x = position_xyz_.x();
            tf.transform.translation.y = position_xyz_.y();
            tf.transform.translation.z = position_xyz_.z();
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_pub_->sendTransform(tf);
        }
    }

    void AppendPathAndPublishLocked(double stamp)
    {
        PublishOdomLocked(stamp);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = get_ros_time(stamp);
        ps.header.frame_id = map_frame_;
        ps.pose.position.x = position_xyz_.x();
        ps.pose.position.y = position_xyz_.y();
        ps.pose.position.z = position_xyz_.z();

        tf2::Quaternion q;
        q.setRPY(x_(0), x_(1), x_(2));
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();

        fused_path_.header = ps.header;
        fused_path_.poses.push_back(ps);
        if (path_pub_)
        {
            path_pub_->publish(fused_path_);
        }
    }

    std::mutex mtx_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    tf2_ros::TransformBroadcaster *tf_pub_ = nullptr;
    std::string map_frame_;
    std::string base_frame_;

    bool initialized_ = false;
    double last_imu_t_ = 0.0;
    double last_meas_t_ = 0.0;
    double history_sec_ = 2.0;
    double max_predict_without_meas_ = 1.0;

    static constexpr double kMaxImuDtSec = 0.2;

    Eigen::Vector3d position_xyz_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 1> x_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> Q_ = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
    std::deque<HistoryEntry> history_;
    nav_msgs::msg::Path fused_path_;
};

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    // cloud_vec	一整帧 Livox 点云，被切成的多个子点云
    // cloud_out	第 i 个子点云（时间片）
    std::cout << "livox_pcl_cbk called." << std::endl;
    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;
    auto shared_msg = std::make_shared<const livox_ros_driver2::msg::CustomMsg>(*msg);

    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(shared_msg, cloud_vec, t_out, false); },
                                   "laser convert");

    for (int i = 0; i < cloud_vec.size(); i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        zjloc::common::Timer::Evaluate([&]()
                                       { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                       "laser ds");
        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]), false);
        // pair<本段数据的绝对起始时间,数据持续时长>
    }
}

void aux_livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    std::cout << "aux_livox_pcl_cbk called." << std::endl;
    std::vector<std::vector<point3D>> cloud_vec; // 附属雷达只存于第0个时间片
    std::vector<double> t_out;                   // 只有第0个时间片，代表总体的时间长度
    auto shared_msg = std::make_shared<const livox_ros_driver2::msg::CustomMsg>(*msg);

    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(shared_msg, cloud_vec, t_out, true); },
                                   "laser convert");

    auto &cloud_out = cloud_vec[0];
    double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
    // double sample_size = 0.01;
    std::mt19937_64 g;
    zjloc::common::Timer::Evaluate([&]()
                                   { std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                   "laser ds");

    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9, t_out[0]), true);
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // sensor_msgs::msg::PointCloud2::SharedPtr cloud(new sensor_msgs::msg::PointCloud2(*msg));

    std::vector<std::vector<point3D>> cloud_vec;
    std::vector<double> t_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_vec, t_out); },
                                   "laser convert");

    for (int i = 0; i < cloud_vec.size(); i++)
    {
        auto &cloud_out = cloud_vec[i];
        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;
        // double sample_size = 0.05;
        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            sub_sample_frame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        },
                                       "laser ds");

        // 在ROS2中使用的是nanoseconds().count()来获取时间戳
        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9 + t_out[i] - t_out[0], t_out[0]), false);
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
    // {
    //     msg_temp->linear_acceleration.x = msg_temp->linear_acceleration.x * gnorm;
    //     msg_temp->linear_acceleration.y = msg_temp->linear_acceleration.y * gnorm;
    //     msg_temp->linear_acceleration.z = msg_temp->linear_acceleration.z * gnorm;
    //     imu_repub->publish(*msg_temp);
    // }
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

    std::string config_file = std::string(ROOT_DIR) + "config/mapping_m.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    // Init Scantext Modules
    scantext_mapping = std::make_shared<scantext::MappingCore>();

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

    auto pub_scan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/scan", 10);
    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            sensor_msgs::msg::PointCloud2::SharedPtr cloud_ptr_output(new sensor_msgs::msg::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = get_ros_time(time);
            cloud_ptr_output->header.frame_id = "odom";
            if (topic_name == "laser")
                pub_scan->publish(*cloud_ptr_output);
            else
                ; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    auto pubLaserOdometry = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
    auto pubLaserOdometryPath = node->create_publisher<nav_msgs::msg::Path>("/odometry_path", 5);

    // 创建tf广播器
    g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    imu_odom_fusion = std::make_shared<ImuOdomFusion>(
        laserOdoPath,
        pubLaserOdometry,
        pubLaserOdometryPath,
        g_tf_broadcaster.get(),
        "odom",
        "aft_mapped");

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            if (topic_name == "laser")
            {
                if (imu_odom_fusion)
                {
                    imu_odom_fusion->OnLioMeasurement(pose, stamp);
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

    std::queue<ScantextTask> scantext_queue;
    std::mutex scantext_queue_mutex;
    std::condition_variable scantext_queue_cv;
    bool stop_scantext_thread = false;

    // Consumer thread function
    std::thread scantext_worker([&]()
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

    Eigen::Isometry3d last_sc_pose = Eigen::Isometry3d::Identity();
    double last_sc_time = 0.0;
    bool has_last_sc = false;

    scantext::ScanContext sc_extractor;

    auto scantext_cbk =
        std::function<bool(const zjloc::CloudPtr &, const SE3 &, double)>(
            [&](const zjloc::CloudPtr &points_world,
                const SE3 &pose,
                double time) -> bool
            {
                if (!scantext_mapping || !points_world)
                    return false;

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
    lio->setFunc(scantext_cbk);

    convert = new zjloc::CloudConvert2;
    convert->LoadFromYAML(config_file);

    lio->setCloudConvert(convert);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string aux_laser_topic = yaml["common"]["aux_lidar_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    gnorm = yaml["common"]["gnorm"].as<double>();

    // 创建订阅者
    std::shared_ptr<void> subLaserCloud =
        (convert->lidar_type_ == zjloc::CloudConvert2::LidarType::AVIA)
            ? std::static_pointer_cast<void>(node->create_subscription<livox_ros_driver2::msg::CustomMsg>(laser_topic, 100, livox_pcl_cbk))
            : std::static_pointer_cast<void>(node->create_subscription<sensor_msgs::msg::PointCloud2>(laser_topic, 100, standard_pcl_cbk));

    auto subAuxLaserCloud = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(aux_laser_topic, 100, aux_livox_pcl_cbk);

    auto sub_imu_ori = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 500, imuHandler);

    std::thread measurement_process(&zjloc::lidarodom_m::run, lio);
    
    rclcpp::spin(node);

    // Cleanup Scantext thread
    {
        std::lock_guard<std::mutex> lock(scantext_queue_mutex);
        stop_scantext_thread = true;
    }
    scantext_queue_cv.notify_all();
    if (scantext_worker.joinable())
        scantext_worker.join();

    rclcpp::shutdown();

    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}
