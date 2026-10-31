#pragma once
#include "algo/imu_odom_predictor.h"
#include "algo/odom_projection.h"
#include <condition_variable>
#include <atomic>
#include <thread>
#include <mutex>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>

// Defined in main_ada.cpp; shared by cloud and odometry publishers.
inline rclcpp::Time get_ros_time(double timestamp);

// ROS adapter. Configuration is immutable after Start(); one worker owns the
// predictor and is the ONLY /odom, path and dynamic TF publisher.
class ImuOdomOutput {
public:
    ImuOdomOutput(const nav_msgs::msg::Path& path,
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& odom,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& path_pub,
        const rclcpp::Clock::SharedPtr& clock, tf2_ros::TransformBroadcaster* tf,
        const std::string& world, const std::string& child)
        : odom_pub_(odom),path_pub_(path_pub),clock_(clock),tf_(tf),
          world_(world),child_(child),path_(path) {}
    ~ImuOdomOutput(){Stop();}
    void SetAftToBaseTransform(const Vec3d& t,const Vec3d& rpy) {
        t_=t; q_=Eigen::AngleAxisd(rpy.z(),Vec3d::UnitZ())*
                 Eigen::AngleAxisd(rpy.y(),Vec3d::UnitY())*Eigen::AngleAxisd(rpy.x(),Vec3d::UnitX());
    }
    void SetBaseLinkFrame(const std::string& frame){base_=frame;}
    void SetMaxPathPoses(size_t count){max_path_=count;}
    void SetStatusPublisher(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub){status_pub_=pub;}
    // Trace is opt-in, buffered, and flushed when the worker is stopped. The
    // steady clock separates sensor coverage from scheduling/transport latency.
    void TraceIngress(double stamp){Trace('C',stamp,0.);}
    void Start(){
        if(const char* path=std::getenv("ADAPTIVE_LIO_OUTPUT_TRACE")) {
            trace_.open(path); trace_<<std::setprecision(17);
            if(!trace_)throw std::runtime_error("cannot open ADAPTIVE_LIO_OUTPUT_TRACE");
        }
        worker_=std::thread([this]{
        try {Run();}
        catch(const std::exception& e){Fail();ReportStatus(nullptr,true);LOG(ERROR)<<"IMU output worker failed: "<<e.what();}
    });}
    void Stop(){
        {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
        cv_.notify_all();if(worker_.joinable())worker_.join();
        std::lock_guard<std::mutex> trace_lock(trace_mutex_);if(trace_.is_open())trace_.flush();
    }
    void SetFaultCallback(std::function<void()> callback){fault_callback_=std::move(callback);}
    void Fail(){if(!faulted_.exchange(true) && fault_callback_)fault_callback_();cv_.notify_one();}
    void OnImu(const IMUPtr& imu){
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || faulted_)return;
        if(imu_queue_.size()>=20000){Fail();return;}
        imu_queue_.push_back(*imu);Trace('I',imu->timestamp_,double(imu_queue_.size()));cv_.notify_one();
    }
    void OnState(const zjloc::LioStateUpdate& state){
        std::lock_guard<std::mutex> lock(mutex_);
        if(stopping_ || faulted_)return;
        Trace('U',state.state.nav.timestamp_,state.last_accepted_lidar_stamp);
        if(!pending_ || state.sequence>pending_->sequence)pending_=state;
        cv_.notify_one();
    }
private:
    void Trace(char event,double stamp,double extra){
        std::lock_guard<std::mutex> lock(trace_mutex_);
        if(trace_.is_open())trace_<<event<<' '<<stamp<<' '<<extra<<' '
            <<std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()<<'\n';
    }
    void ReportStatus(zjloc::ImuOdomPredictor* predictor,bool force=false){
        if(!status_pub_)return;
        const auto now=std::chrono::steady_clock::now();
        const bool input_stale=now-last_input_>std::chrono::milliseconds(500);
        const bool fault=faulted_ || (predictor && predictor->Faulted());
        const bool valid=predictor && predictor->Publishable();
        const std::string state=fault?"FAULT":!valid?"UNINITIALIZED":
            (input_stale || predictor->Degraded()?"DEGRADED":"FRESH");
        if(!force && state==last_health_ && now-last_status_<std::chrono::milliseconds(100))return;
        const double age=predictor?predictor->CorrectionAge():std::numeric_limits<double>::infinity();
        std::ostringstream json;json<<std::setprecision(17)
            <<"{\"state\":\""<<state<<"\",\"stamp\":"<<last_stamp_
            <<",\"correction_age_s\":";
        if(std::isfinite(age))json<<age;else json<<"null";
        json<<",\"input_stale\":"<<(input_stale?"true":"false")
            <<",\"history_miss\":"<<(predictor?predictor->counters.history_miss:0)
            <<",\"last_publish_sensor_interval_s\":"<<last_sensor_interval_<<"}";
        std_msgs::msg::String msg;msg.data=json.str();
        // Shutdown may invalidate the ROS context before the worker can report.
        try {status_pub_->publish(msg);} catch(const std::exception& e){
            LOG(WARNING)<<"status publication failed: "<<e.what();
        }
        last_status_=now;last_health_=state;
    }
    void Run(){
        zjloc::ImuOdomPredictor predictor;
        double last_published=-std::numeric_limits<double>::infinity(),last_path=last_published;
        auto last_log=std::chrono::steady_clock::now();
        last_input_=last_log;
        auto advance=[&]{
            const bool replay=predictor.HasReadyUpdate();
            const double anchor=predictor.PendingStamp();
            if(replay)Trace('R',anchor,predictor.LastAccepted());
            predictor.Advance();
            if(replay)Trace('E',anchor,predictor.LastAccepted());
        };
        auto publish_prediction=[&]{
            if(!predictor.Publishable() || faulted_)return;
            const auto state=predictor.Snapshot();
            const double stamp=state.nav.timestamp_;
            // Downsample by real sensor time; never restamp or interpolate an
            // absent IMU sample. Check every sample, including catch-up bursts.
            if(stamp<=last_published || stamp-last_published<.02-1e-6)return;
            const auto projected=zjloc::ProjectOdometry(state,predictor.AngularVelocity(),predictor.GyroVariance());
            nav_msgs::msg::Odometry odom;
            odom.header.stamp=get_ros_time(stamp);odom.header.frame_id=world_;odom.child_frame_id=child_;
            const auto& pos=projected.pose.translation();const auto q=projected.pose.unit_quaternion();
            odom.pose.pose.position.x=pos.x();odom.pose.pose.position.y=pos.y();odom.pose.pose.position.z=pos.z();
            odom.pose.pose.orientation.x=q.x();odom.pose.pose.orientation.y=q.y();
            odom.pose.pose.orientation.z=q.z();odom.pose.pose.orientation.w=q.w();
            odom.twist.twist.linear.x=projected.linear.x();odom.twist.twist.linear.y=projected.linear.y();odom.twist.twist.linear.z=projected.linear.z();
            odom.twist.twist.angular.x=projected.angular.x();odom.twist.twist.angular.y=projected.angular.y();odom.twist.twist.angular.z=projected.angular.z();
            for(int r=0;r<6;++r)for(int c=0;c<6;++c){
                odom.pose.covariance[r*6+c]=projected.pose_cov(r,c);
                odom.twist.covariance[r*6+c]=projected.twist_cov(r,c);
            }
            Trace('P',stamp,predictor.CorrectionAge());
            odom_pub_->publish(odom);
            Trace('D',stamp,predictor.CorrectionAge());
            geometry_msgs::msg::TransformStamped tf;
            tf.header=odom.header;tf.child_frame_id=child_;tf.transform.translation.x=pos.x();
            tf.transform.translation.y=pos.y();tf.transform.translation.z=pos.z();tf.transform.rotation=odom.pose.pose.orientation;
            geometry_msgs::msg::TransformStamped fixed;
            fixed.header=odom.header;fixed.header.frame_id=child_;fixed.child_frame_id=base_;
            fixed.transform.translation.x=t_.x();fixed.transform.translation.y=t_.y();fixed.transform.translation.z=t_.z();
            fixed.transform.rotation.x=q_.x();fixed.transform.rotation.y=q_.y();fixed.transform.rotation.z=q_.z();fixed.transform.rotation.w=q_.w();
            if(tf_)tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{tf,fixed});
            last_sensor_interval_=std::isfinite(last_published)?stamp-last_published:0.;
            last_published=stamp; last_stamp_=stamp;
            if(max_path_ && path_pub_->get_subscription_count()>0 && stamp-last_path>=.1){
                geometry_msgs::msg::PoseStamped ps;ps.header=odom.header;ps.pose=odom.pose.pose;
                path_.header=odom.header;path_.poses.push_back(ps);
                if(path_.poses.size()>max_path_)path_.poses.erase(path_.poses.begin(),path_.poses.end()-max_path_);
                path_pub_->publish(path_);last_path=stamp;
            }
        };
        while(true){
            std::deque<zjloc::IMU> batch;std::optional<zjloc::LioStateUpdate> update;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock,std::chrono::milliseconds(100),[&]{return stopping_||faulted_||!imu_queue_.empty()||pending_.has_value();});
                if(stopping_)break;
                if(faulted_){lock.unlock();ReportStatus(&predictor,true);break;}
                for(size_t i=0;i<4 && !imu_queue_.empty();++i){
                    batch.push_back(std::move(imu_queue_.front()));imu_queue_.pop_front();
                }
                update=std::move(pending_);pending_.reset();
            }
            if(update){
                predictor.Submit(*update);advance();
            }
            if(!batch.empty())last_input_=std::chrono::steady_clock::now();
            for(const auto& imu:batch){
                if(faulted_)break;
                predictor.Append(imu);advance();
                if(predictor.Faulted()){Fail();break;}
                publish_prediction();
            }
            if(predictor.Faulted())Fail();
            ReportStatus(&predictor);
            if(faulted_)break;
            const auto now=std::chrono::steady_clock::now();
            if(now-last_log>std::chrono::seconds(5)){
                const auto& c=predictor.counters;
                LOG(INFO)<<"imu_output valid="<<predictor.Publishable()<<" history="<<predictor.HistorySize()
                    <<" applied="<<c.applied<<" stale_state="<<c.stale_state
                    <<" history_miss="<<c.history_miss<<" stale_checks="<<c.timeout
                    <<" degraded="<<predictor.Degraded()<<" correction_age_s="<<predictor.CorrectionAge()
                    <<" batch="<<batch.size();last_log=now;
            }
        }
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std::ofstream trace_;std::mutex trace_mutex_;
    std::chrono::steady_clock::time_point last_input_{},last_status_{};
    std::string last_health_;
    double last_stamp_=0.,last_sensor_interval_=0.;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Clock::SharedPtr clock_;
    tf2_ros::TransformBroadcaster* tf_;
    std::string world_,child_,base_="base_link";
    Vec3d t_=Vec3d::Zero();Eigen::Quaterniond q_=Eigen::Quaterniond::Identity();
    nav_msgs::msg::Path path_;size_t max_path_=300;
    std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
    std::deque<zjloc::IMU> imu_queue_;std::optional<zjloc::LioStateUpdate> pending_;
    std::function<void()> fault_callback_;
    bool stopping_=false;std::atomic<bool> faulted_{false};
};
