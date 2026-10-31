// Exercises the production ROS output adapter and DDS subscriber delivery.
// Sensor-time lag is injected by feeding historical posteriors; it is not a
// wall-clock real-time performance benchmark or a LiDAR registration test.
#include "apps/imu_odom_output.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <vector>

inline rclcpp::Time get_ros_time(double timestamp) {
    return rclcpp::Time(static_cast<int64_t>(std::llround(timestamp * 1e9)));
}
namespace {
using namespace std::chrono_literals;
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
zjloc::LioStateUpdate posterior(double stamp, uint64_t sequence) {
    zjloc::ESKFD filter;
    auto state = filter.ExportSnapshot();
    state.nav.timestamp_ = stamp;
    state.gravity = Vec3d(0, 0, -9.81);
    return {state, sequence, stamp, Vec3d::Constant(1e-5)};
}
struct Harness {
    rclcpp::Node::SharedPtr node;
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
    std::unique_ptr<ImuOdomOutput> output;
    std::thread spin;
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<nav_msgs::msg::Odometry> odom;
    std::vector<std::string> statuses;
    bool callback_finite = true;
    explicit Harness(const std::string& name) {
        node = std::make_shared<rclcpp::Node>(name);
        const std::string prefix = "/test_" + name;
        odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(prefix + "/odom", 10000);
        path_pub = node->create_publisher<nav_msgs::msg::Path>(prefix + "/path", 1);
        status_pub = node->create_publisher<std_msgs::msg::String>(prefix + "/status", rclcpp::QoS(1).reliable().transient_local());
        odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(prefix + "/odom", 10000,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                std::lock_guard<std::mutex> lock(mutex);
                const auto& p = message->pose.pose.position;
                const auto& q = message->pose.pose.orientation;
                callback_finite &= std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)
                    && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
                for (double value : message->pose.covariance) callback_finite &= std::isfinite(value);
                for (double value : message->twist.covariance) callback_finite &= std::isfinite(value);
                odom.push_back(*message); cv.notify_all();
            });
        status_sub = node->create_subscription<std_msgs::msg::String>(prefix + "/status",
            rclcpp::QoS(100).reliable().transient_local(), [this](std_msgs::msg::String::ConstSharedPtr message) {
                std::lock_guard<std::mutex> lock(mutex); statuses.push_back(message->data); cv.notify_all();
            });
        output = std::make_unique<ImuOdomOutput>(nav_msgs::msg::Path{}, odom_pub, path_pub, node->get_clock(), nullptr, "odom", "aft_mapped");
        output->SetStatusPublisher(status_pub);
        output->SetMaxPathPoses(0);
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while ((odom_pub->get_subscription_count() == 0 || status_pub->get_subscription_count() == 0)
               && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(10ms);
        require(odom_pub->get_subscription_count() > 0 && status_pub->get_subscription_count() > 0,
                "DDS subscriptions did not match");
        executor.add_node(node);
        spin = std::thread([this] { executor.spin(); });
        output->Start();
    }
    ~Harness() {
        output->Stop(); executor.cancel();
        if (spin.joinable()) spin.join();
    }
    static double stamp(const nav_msgs::msg::Odometry& value) {
        return value.header.stamp.sec + value.header.stamp.nanosec * 1e-9;
    }
    void sample(double timestamp) {
        output->OnImu(std::make_shared<zjloc::IMU>(timestamp, Vec3d::Zero(), Vec3d(0, 0, 9.81)));
    }
    void initialize() {
        // Initialize before building a backlog; initialization holes are measured
        // separately from steady-state output continuity.
        output->OnState(posterior(1, 1)); sample(1); wait_stamp(1);
    }
    void wait_stamp(double target) {
        std::unique_lock<std::mutex> lock(mutex);
        require(cv.wait_for(lock, 5s, [&] { return !odom.empty() && stamp(odom.back()) >= target - 1e-7; }),
                "timeout waiting for actual ROS odom sample " + std::to_string(target));
    }
    void wait_status(const std::string& state, bool input_stale = false) {
        std::unique_lock<std::mutex> lock(mutex);
        require(cv.wait_for(lock, 3s, [&] {
            return !statuses.empty() && statuses.back().find("\"state\":\"" + state + "\"") != std::string::npos
                && (!input_stale || statuses.back().find("\"input_stale\":true") != std::string::npos);
        }), "timeout waiting for ROS health state " + state);
    }
    size_t count() { std::lock_guard<std::mutex> lock(mutex); return odom.size(); }
    void check_continuity(const std::string& label) {
        std::lock_guard<std::mutex> lock(mutex);
        require(callback_finite, "nonfinite output pose/covariance");
        require(odom.size() > 100, "too few output samples");
        double maximum_gap = 0;
        for (size_t i = 0; i < odom.size(); ++i) {
            const double t = stamp(odom[i]);
            // Every publication must belong to the actual 200Hz sensor grid.
            require(std::abs((t - 1) / .005 - std::round((t - 1) / .005)) < 1e-5,
                    "output timestamp not on actual IMU grid");
            if (!i) continue;
            const double gap = t - stamp(odom[i-1]);
            require(gap > 0, "duplicate/backward publication timestamp");
            maximum_gap = std::max(maximum_gap, gap);
        }
        require(maximum_gap <= .025001, "sensor-time output gap exceeds 25ms");
        std::cout << "PASS " << label << " received=" << odom.size() << " max_sensor_gap_s=" << maximum_gap
                  << " finite=1 duplicates=0 backwards=0\n";
    }
};
void delay_case(double delay, const std::string& name) {
    Harness h(name); h.initialize();
    uint64_t sequence = 1;
    for (int block = 1; block <= 50; ++block) {
        const double end = 1 + block * .1;
        if (end - delay > 1) h.output->OnState(posterior(end - delay, ++sequence));
        for (int i = (block-1)*20 + 1; i <= block*20; ++i) h.sample(1 + i*.005);
        h.wait_stamp(end - .020001);
    }
    h.wait_status("DEGRADED");
    if (delay > 2) {
        std::unique_lock<std::mutex> lock(h.mutex);
        require(h.cv.wait_for(lock, 2s, [&] {
            return !h.statuses.empty() && h.statuses.back().find("\"history_miss\":0") == std::string::npos;
        }), "expired posterior not reported");
    }
    h.output->OnState(posterior(6, ++sequence));
    h.sample(6.005); h.sample(6.010); h.sample(6.015); h.sample(6.020);
    h.wait_stamp(6.020 - 1e-7); h.wait_status("FRESH");
    h.check_continuity(name);
}
void burst_and_silence() {
    Harness h("burst"); h.initialize();
    for (int i = 1; i <= 800; ++i) h.sample(1 + i*.005);
    h.wait_stamp(5); h.check_continuity("burst_4_sensor_seconds");
    h.output->OnState(posterior(5, 2)); h.wait_status("FRESH");
    const size_t before = h.count();
    h.wait_status("DEGRADED", true);
    require(h.count() == before, "input silence fabricated odometry");
    std::cout << "PASS silence_input_stale=1 synthetic_odom=0\n";
}
void fault_case(bool nonfinite) {
    Harness h(nonfinite ? "nan" : "gap"); h.initialize();
    for (int i = 1; i <= 20; ++i) h.sample(1 + i*.005);
    h.wait_stamp(1.1);
    if (nonfinite) {
        auto bad = std::make_shared<zjloc::IMU>(1.105, Vec3d::Zero(), Vec3d(0, 0, 9.81));
        bad->acce_.x() = std::numeric_limits<double>::quiet_NaN(); h.output->OnImu(bad);
    } else h.sample(1.18);
    h.wait_status("FAULT");
    const size_t count = h.count();
    h.sample(1.185); h.output->OnState(posterior(1.185, 2));
    std::this_thread::sleep_for(150ms);
    require(h.count() == count, "latched fault resumed output");
    std::cout << "PASS " << (nonfinite ? "nonfinite" : "80ms_gap") << " fault_status=1 post_fault_odom=0\n";
}
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        delay_case(.75, "delay075"); delay_case(1.5, "delay150"); delay_case(2.5, "delay250");
        burst_and_silence(); fault_case(false); fault_case(true);
        std::cout << "ALL ROS OUTPUT ADAPTER TESTS PASSED\n";
    } catch (const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; result = 1; }
    rclcpp::shutdown(); return result;
}
