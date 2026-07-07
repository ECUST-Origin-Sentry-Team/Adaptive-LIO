#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mapping
{

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;

/**
 * @brief One recorded LiDAR frame for static map generation / ERASOR2 export.
 *
 * The stored cloud is in the same local frame as pose.  In the current
 * Adaptive-LIO exporter this is the IMU/body frame: p_map = pose * p_local.
 */
struct KeyFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int index = -1;
    double time = 0.0;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    std::string cloud_path;
    PointCloudPtr cloud;
};

/**
 * @brief Minimal mapping recorder.
 *
 * This class only supports:
 *   1. optional global PCD map accumulation/saving;
 *   2. ERASOR2/KITTI-style per-frame scan export.
 */
class MappingCore
{
public:
    struct Config
    {
        bool enable_mapping = false;

        // ERASOR2 / KITTI style dataset export.
        // Output layout:
        //   <erasor2_save_dir>/dataset/sequences/<sequence_id>/velodyne/000000.bin
        //   <erasor2_save_dir>/dataset/sequences/<sequence_id>/poses_suma_optim.txt
        //   <erasor2_save_dir>/dataset/sequences/<sequence_id>/times.txt
        bool export_erasor2 = true;
        std::string erasor2_save_dir = "./erasor2_dataset";
        std::string sequence_id = "00";
        bool record_every_frame = true;

        // Used only when record_every_frame == false.
        double keyframe_dist_thresh = 1.0;  // meters
        double keyframe_angle_thresh = 0.2; // radians

        // Optional accumulated global PCD map output.
        bool save_global_map = false;
        std::string map_save_path = "./map.pcd";
        double auto_save_interval = 0.0; // meters, 0 disables auto-save
    };

    MappingCore();
    ~MappingCore() = default;

    void setConfig(const Config &config);
    Config getConfig() const { return config_; }

    void initOutputIfNeeded();

    /**
     * @brief Record one LiDAR scan and its pose.
     * @param cloud Local scan in the same frame as pose. Only x/y/z/intensity are exported.
     * @param pose T_map_local corresponding to this scan.
     * @param time Absolute timestamp in seconds. times.txt stores time relative to first recorded frame.
     * @param out_kf Optional recorded-frame metadata.
     */
    bool addFrame(const PointCloudPtr &cloud,
                  const Eigen::Isometry3d &pose,
                  double time,
                  std::shared_ptr<KeyFrame> *out_kf = nullptr);

    bool saveMap(const std::string &path);
    void saveMapAsync(const std::string &path);

    std::vector<std::shared_ptr<KeyFrame>> getKeyFrames();

private:
    bool shouldRecordFrame(const Eigen::Isometry3d &pose) const;
    bool appendErasor2Frame(const PointCloudType &cloud,
                            const Eigen::Isometry3d &pose,
                            double time,
                            int index,
                            std::string *relative_cloud_path);
    void appendToGlobalMap(const PointCloudType &cloud,
                           const Eigen::Isometry3d &pose);

    Config config_;

    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    mutable std::mutex map_mutex_;

    Eigen::Isometry3d last_recorded_pose_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d last_auto_save_pose_ = Eigen::Isometry3d::Identity();
    bool has_recorded_pose_ = false;
    bool has_auto_save_pose_ = false;

    std::atomic<bool> saving_{false};
    PointCloudPtr global_map_;

    std::string erasor2_sequence_dir_;
    bool output_initialized_{false};

    int frame_index_{0};
    double first_frame_time_{0.0};
    bool has_first_frame_time_{false};
};

} // namespace mapping
