#include "mapping_module/Mapping.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace mapping
{
namespace
{

std::string kittiBinFileName(int index)
{
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << index << ".bin";
    return oss.str();
}

bool writeKittiBin(const std::string &path, const PointCloudType &cloud)
{
    const std::string tmp_path = path + ".tmp";
    std::ofstream os(tmp_path, std::ios::binary | std::ios::trunc);
    if (!os.is_open())
    {
        return false;
    }

    for (const auto &p : cloud.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {
            continue;
        }

        const float xyzi[4] = {
            static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z),
            std::isfinite(p.intensity) ? static_cast<float>(p.intensity) : 0.0f};
        os.write(reinterpret_cast<const char *>(xyzi), sizeof(xyzi));
    }

    os.close();
    if (!os)
    {
        std::filesystem::remove(tmp_path);
        return false;
    }

    std::error_code ec;
    std::filesystem::rename(tmp_path, path, ec);
    if (ec)
    {
        std::filesystem::remove(path, ec);
        std::filesystem::rename(tmp_path, path, ec);
    }
    return !ec;
}

void writeKittiPoseLine(std::ostream &os, const Eigen::Isometry3d &pose)
{
    const Eigen::Matrix3d R = pose.rotation();
    const Eigen::Vector3d t = pose.translation();
    os << std::fixed << std::setprecision(9)
       << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t.x() << " "
       << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t.y() << " "
       << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t.z() << "\n";
}

} // namespace

MappingCore::MappingCore()
{
    global_map_ = pcl::make_shared<PointCloudType>();
}

void MappingCore::setConfig(const Config &config)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    config_ = config;
    output_initialized_ = false;
    frame_index_ = 0;
    keyframes_.clear();
    global_map_->clear();
    has_recorded_pose_ = false;
    has_auto_save_pose_ = false;
    has_first_frame_time_ = false;
}

void MappingCore::initOutputIfNeeded()
{
    if (output_initialized_)
    {
        return;
    }

    if (config_.export_erasor2)
    {
        const std::filesystem::path root(config_.erasor2_save_dir);
        erasor2_sequence_dir_ = (root / "dataset" / "sequences" / config_.sequence_id).string();
        const std::filesystem::path velodyne_dir = std::filesystem::path(erasor2_sequence_dir_) / "velodyne";

        std::filesystem::create_directories(velodyne_dir);

        // Start a fresh trajectory log for each run.
        {
            std::ofstream pose_ofs(erasor2_sequence_dir_ + "/poses_suma_optim.txt", std::ios::trunc);
            if (!pose_ofs.is_open())
            {
                std::cerr << "[Mapping] Failed to create poses_suma_optim.txt under "
                          << erasor2_sequence_dir_ << std::endl;
            }
        }
        {
            std::ofstream time_ofs(erasor2_sequence_dir_ + "/times.txt", std::ios::trunc);
            if (!time_ofs.is_open())
            {
                std::cerr << "[Mapping] Failed to create times.txt under "
                          << erasor2_sequence_dir_ << std::endl;
            }
        }

        std::cout << "[Mapping] ERASOR2 export enabled: " << erasor2_sequence_dir_ << std::endl;
    }

    output_initialized_ = true;
}

bool MappingCore::shouldRecordFrame(const Eigen::Isometry3d &pose) const
{
    if (config_.record_every_frame || !has_recorded_pose_)
    {
        return true;
    }

    const Eigen::Isometry3d delta = last_recorded_pose_.inverse() * pose;
    const double dist = delta.translation().norm();
    const double ang = Eigen::AngleAxisd(delta.rotation()).angle();
    return dist >= config_.keyframe_dist_thresh || ang >= config_.keyframe_angle_thresh;
}

bool MappingCore::appendErasor2Frame(const PointCloudType &cloud,
                                     const Eigen::Isometry3d &pose,
                                     double time,
                                     int index,
                                     std::string *relative_cloud_path)
{
    if (!config_.export_erasor2)
    {
        return true;
    }

    const std::string file_name = kittiBinFileName(index);
    const std::string rel_path = "velodyne/" + file_name;
    const std::string bin_path = erasor2_sequence_dir_ + "/" + rel_path;

    if (!writeKittiBin(bin_path, cloud))
    {
        std::cerr << "[Mapping] Failed to write ERASOR2 bin: " << bin_path << std::endl;
        return false;
    }

    if (!has_first_frame_time_)
    {
        first_frame_time_ = time;
        has_first_frame_time_ = true;
    }

    {
        std::ofstream pose_ofs(erasor2_sequence_dir_ + "/poses_suma_optim.txt", std::ios::app);
        if (!pose_ofs.is_open())
        {
            std::cerr << "[Mapping] Failed to append poses_suma_optim.txt" << std::endl;
            return false;
        }
        writeKittiPoseLine(pose_ofs, pose);
    }

    {
        std::ofstream time_ofs(erasor2_sequence_dir_ + "/times.txt", std::ios::app);
        if (!time_ofs.is_open())
        {
            std::cerr << "[Mapping] Failed to append times.txt" << std::endl;
            return false;
        }
        time_ofs << std::fixed << std::setprecision(9) << (time - first_frame_time_) << "\n";
    }

    if (relative_cloud_path)
    {
        *relative_cloud_path = rel_path;
    }
    return true;
}

void MappingCore::appendToGlobalMap(const PointCloudType &cloud,
                                    const Eigen::Isometry3d &pose)
{
    if (!config_.save_global_map || cloud.empty())
    {
        return;
    }

    PointCloudType transformed;
    pcl::transformPointCloud(cloud, transformed, pose.matrix().cast<float>());
    *global_map_ += transformed;
}

bool MappingCore::addFrame(const PointCloudPtr &cloud,
                           const Eigen::Isometry3d &pose,
                           double time,
                           std::shared_ptr<KeyFrame> *out_kf)
{
    if (!cloud)
    {
        return false;
    }

    std::shared_ptr<KeyFrame> kf;
    bool need_auto_save = false;
    std::string auto_save_path;

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!config_.enable_mapping)
        {
            return true;
        }

        initOutputIfNeeded();

        if (!shouldRecordFrame(pose))
        {
            return true;
        }

        const int index = frame_index_++;
        std::string rel_cloud_path;
        if (!appendErasor2Frame(*cloud, pose, time, index, &rel_cloud_path))
        {
            return false;
        }

        appendToGlobalMap(*cloud, pose);

        kf = std::make_shared<KeyFrame>();
        kf->index = index;
        kf->time = time;
        kf->pose = pose;
        kf->cloud_path = rel_cloud_path;
        if (config_.save_global_map)
        {
            kf->cloud = cloud;
        }
        keyframes_.push_back(kf);

        if (config_.auto_save_interval > 0.0 && config_.save_global_map)
        {
            if (!has_auto_save_pose_)
            {
                has_auto_save_pose_ = true;
                last_auto_save_pose_ = pose;
            }
            else
            {
                const double dist = (pose.translation() - last_auto_save_pose_.translation()).norm();
                if (dist >= config_.auto_save_interval)
                {
                    need_auto_save = true;
                    auto_save_path = config_.map_save_path;
                    last_auto_save_pose_ = pose;
                }
            }
        }

        last_recorded_pose_ = pose;
        has_recorded_pose_ = true;
    }

    if (out_kf)
    {
        *out_kf = kf;
    }

    if (need_auto_save)
    {
        saveMapAsync(auto_save_path);
    }

    return true;
}

bool MappingCore::saveMap(const std::string &path)
{
    PointCloudPtr map_copy(new PointCloudType);
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        *map_copy = *global_map_;
    }

    if (map_copy->empty())
    {
        std::cerr << "[Mapping] Global map is empty; skip save: " << path << std::endl;
        return false;
    }

    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty())
    {
        std::filesystem::create_directories(parent);
    }

    if (pcl::io::savePCDFileBinary(path, *map_copy) != 0)
    {
        std::cerr << "[Mapping] Failed to save map to " << path << std::endl;
        return false;
    }

    std::cout << "[Mapping] Saved map with " << map_copy->size() << " points to " << path << std::endl;
    return true;
}

void MappingCore::saveMapAsync(const std::string &path)
{
    if (saving_.exchange(true))
    {
        std::cout << "[Mapping] Save already in progress, skip request." << std::endl;
        return;
    }

    std::thread([this, path]()
                {
                    std::cout << "[Mapping] Async saving map..." << std::endl;
                    const bool ok = saveMap(path);
                    std::cout << (ok ? "[Mapping] Async map saved." : "[Mapping] Async save failed.") << std::endl;
                    saving_ = false;
                })
        .detach();
}

std::vector<std::shared_ptr<KeyFrame>> MappingCore::getKeyFrames()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    return keyframes_;
}

} // namespace mapping
