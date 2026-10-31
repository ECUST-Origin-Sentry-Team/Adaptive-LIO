#include "algo/imu_odom_predictor.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace zjloc;
namespace {
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
IMU imu(double stamp) { return IMU(stamp, Vec3d::Zero(), Vec3d(0, 0, 9.81)); }
LioStateUpdate posterior(double stamp, uint64_t sequence) {
    ESKFD filter;
    auto state = filter.ExportSnapshot();
    state.nav.timestamp_ = stamp;
    state.gravity = Vec3d(0, 0, -9.81);
    return {state, sequence, stamp, Vec3d::Constant(1e-5)};
}
void append(ImuOdomPredictor& predictor, double stamp) {
    require(predictor.Append(imu(stamp)), "valid IMU rejected");
    predictor.Advance();
}
void delayed_stream(double delay) {
    ImuOdomPredictor predictor;
    require(predictor.Submit(posterior(1, 1)), "initial posterior rejected");
    uint64_t sequence = 1;
    size_t stale = 0;
    for (int i = 0; i <= 1000; ++i) {
        const double stamp = 1 + i * .005;
        // Model regularly arriving posteriors with a fixed sensor-time lag.
        if (i % 20 == 0 && stamp - delay > 1)
            require(predictor.Submit(posterior(stamp - delay, ++sequence)), "delayed posterior rejected on submission");
        append(predictor, stamp);
        require(predictor.Publishable(), "valid IMU prediction stopped because correction was late");
        require(std::abs(predictor.Snapshot().nav.timestamp_ - stamp) < 1e-12, "prediction stamp is not real IMU time");
        stale += predictor.CorrectionStale();
    }
    require(stale > 0, "delay did not exercise degradation");
    if (delay < 2) {
        require(predictor.counters.applied > 1, "in-window delayed correction not applied");
        require(predictor.counters.history_miss == 0, "in-window delayed correction discarded");
    } else {
        require(predictor.counters.history_miss > 0, "expired correction not counted");
        require(predictor.counters.applied == 1, "expired correction applied at wrong time");
    }
    require(predictor.Submit(posterior(6, ++sequence)), "fresh recovery posterior rejected");
    predictor.Advance();
    require(predictor.Publishable() && !predictor.CorrectionStale() && !predictor.Degraded(), "fresh correction did not recover");
    std::cout << "PASS delay_s=" << delay << " valid_checks=1001 stale_checks=" << stale
              << " history_miss=" << predictor.counters.history_miss << " fresh_recovery=1\n";
}
void replay_equivalence(double delay) {
    ImuOdomPredictor timely, late;
    timely.Submit(posterior(1, 1));
    late.Submit(posterior(1, 1));
    auto correction = posterior(1.202, 2); // lies within a raw IMU interval
    correction.state.nav.p_ = Vec3d(.2, -.1, .05);
    correction.state.nav.v_ = Vec3d(.3, -.2, .1);
    correction.state.nav.bg_ = Vec3d(.01, .02, -.03);
    bool sent = false;
    for (int i = 0; i <= 400; ++i) {
        const double stamp = 1 + i * .005;
        if (i == 40) timely.Submit(correction); // future posterior must wait for IMU coverage
        if (!sent && stamp >= correction.state.nav.timestamp_ + delay) {
            late.Submit(correction);
            sent = true;
        }
        IMU sample(stamp, Vec3d(.2, -.1, .3), Vec3d(.1 * std::sin(stamp), .2, 9.81));
        require(timely.Append(sample) && late.Append(sample), "replay IMU rejected");
        timely.Advance();
        late.Advance();
    }
    require(sent, "replay correction never delivered");
    const auto a = timely.Snapshot(), b = late.Snapshot();
    require((a.nav.p_ - b.nav.p_).norm() < 1e-11, "late replay position differs from timely correction");
    require((a.nav.v_ - b.nav.v_).norm() < 1e-11, "late replay velocity differs");
    require((a.nav.R_.inverse() * b.nav.R_).log().norm() < 1e-11, "late replay attitude differs");
    require((a.covariance - b.covariance).norm() < 1e-11, "late replay covariance differs");
    std::cout << "PASS off_grid_replay_delay_s=" << delay << " state_and_covariance_equal=1\n";
}
}
int main() {
    try {
        for (double delay : {.75, 1.5, 2.5}) delayed_stream(delay);
        for (double delay : {.75, 1.5}) replay_equivalence(delay);
        ImuOdomPredictor burst;
        burst.Submit(posterior(1, 1));
        append(burst, 1);
        const double initial_covariance_trace = burst.Snapshot().covariance.trace();
        // A backlog greater than history duration is consumed in chronological order.
        // The ROS adapter must call Append + Advance for each sample, not Append all first.
        for (int i = 1; i <= 800; ++i) append(burst, 1 + i * .005);
        require(burst.Publishable() && burst.CorrectionStale(), "four-second catch-up stopped");
        require(burst.HistorySize() <= 403, "history storage unbounded");
        require(burst.Snapshot().covariance.trace() > initial_covariance_trace, "uncorrected covariance did not grow");
        const auto before = burst.Snapshot();
        require(!burst.Append(imu(4.99)), "reordered IMU accepted");
        require(!burst.Append(imu(5)), "duplicate IMU accepted");
        require(!burst.Faulted(), "duplicate or minor reorder unnecessarily faulted");
        require(!burst.Submit(posterior(.9, 2)), "older posterior anchor accepted");
        auto invalid = posterior(5, 2);
        invalid.state.covariance(0, 0) = -1;
        require(!burst.Submit(invalid), "invalid posterior covariance accepted");
        require((burst.Snapshot().covariance - before.covariance).norm() == 0, "rejected input mutated state");
        append(burst, 5.08);
        require(burst.Faulted() && !burst.Publishable(), "real 80ms IMU gap did not fault");
        require(!burst.Submit(posterior(5.08, 3)), "LiDAR posterior cleared a latched IMU fault");
        std::cout << "PASS burst_s=4 covariance_growth=1 ordering_rejection=1 gap_fault_latched=1\n";
        for (bool bad_time : {false, true}) {
            ImuOdomPredictor invalid_imu;
            invalid_imu.Submit(posterior(1, 1));
            append(invalid_imu, 1);
            auto sample = imu(1.005);
            if (bad_time) sample.timestamp_ = std::numeric_limits<double>::quiet_NaN();
            else sample.acce_.x() = std::numeric_limits<double>::quiet_NaN();
            require(!invalid_imu.Append(sample), "nonfinite IMU accepted");
            require(invalid_imu.Faulted() && !invalid_imu.Publishable(), "nonfinite IMU did not fault");
        }
        ImuOdomPredictor delayed_init;
        for (int i = 0; i <= 600; ++i) append(delayed_init, 1 + i * .005);
        delayed_init.Submit(posterior(1, 1));
        delayed_init.Advance();
        require(!delayed_init.Faulted() && !delayed_init.Publishable(), "expired initialization permanently faulted");
        delayed_init.Submit(posterior(4, 2));
        delayed_init.Advance();
        require(delayed_init.Publishable(), "initialization failed after expired initial posterior");
        ImuOdomPredictor ordered;
        ordered.Submit(posterior(1, 1));
        append(ordered, 1);
        require(ordered.Submit(posterior(1.1, 3)), "future posterior rejected");
        require(!ordered.Submit(posterior(1.05, 2)), "pending sequence regressed");
        require(!ordered.Submit(posterior(1.05, 4)), "pending anchor time regressed");
        auto regressed_age = posterior(1.2, 4);
        regressed_age.last_accepted_lidar_stamp = 1;
        require(!ordered.Submit(regressed_age), "pending accepted correction time regressed");
        for (int i = 1; i <= 20; ++i) append(ordered, 1 + i * .005);
        require(std::abs(ordered.LastAccepted() - 1.1) < 1e-12, "rejected update replaced valid pending correction");
        ImuOdomPredictor replay_failure;
        replay_failure.Submit(posterior(1, 1));
        for (int i = 0; i <= 20; ++i) append(replay_failure, 1 + i * .005);
        auto incompatible = posterior(1.05, 2);
        incompatible.state.options.imu_dt_ = .0001; // valid snapshot, incompatible propagation step
        require(replay_failure.Submit(incompatible), "test snapshot was not valid");
        replay_failure.Advance();
        require(replay_failure.Faulted() && !replay_failure.Publishable(), "partial failed replay exposed as valid output");
        std::cout << "PASS nonfinite_fault=1 expired_initialization_recovery=1 pending_ordering=1 replay_failure_closed=1\nALL CONTINUITY TESTS PASSED\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
