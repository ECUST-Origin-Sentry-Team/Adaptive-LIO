// Compile this identical source against baseline and repaired headers.
// Nonzero failure counts are observed regression evidence, not test-run errors.
#include "algo/imu_odom_predictor.h"
#include <cstdlib>
#include <iomanip>
#include <iostream>
using namespace zjloc;
namespace {
IMU sample(double t) { return IMU(t, Vec3d::Zero(), Vec3d(0, 0, 9.81)); }
LioStateUpdate correction(double t, uint64_t sequence) {
    ESKFD filter;
    auto state = filter.ExportSnapshot();
    state.nav.timestamp_ = t;
    state.gravity = Vec3d(0, 0, -9.81);
    return {state, sequence, t, Vec3d::Constant(1e-5)};
}
void feed(ImuOdomPredictor& predictor, double t) {
    predictor.Append(sample(t));
    predictor.Advance();
}
void delay(double lag, double limit) {
    ImuOdomPredictor p(2, limit);
    p.Submit(correction(1, 1));
    uint64_t sequence = 1;
    unsigned publishable = 0, stopped = 0;
    double last_output = 1, max_gap = 0;
    for (int i = 0; i <= 1000; ++i) {
        const double t = 1 + i * .005;
        if (i % 20 == 0 && t - lag > 1) p.Submit(correction(t - lag, ++sequence));
        feed(p, t);
        if (p.Publishable()) {
            ++publishable;
            max_gap = std::max(max_gap, t - last_output);
            last_output = t;
        } else ++stopped;
    }
    // Include the censored interval at end of stream; otherwise stopped outputs
    // would deceptively report a small maximum inter-message gap.
    const double tail_silence = 6 - last_output;
    const bool fault_before = p.Faulted();
    p.Submit(correction(6, ++sequence));
    p.Advance();
    std::cout << "{\"lag_s\":" << lag << ",\"checks\":1001,\"publishable\":" << publishable
              << ",\"suppressed\":" << stopped << ",\"history_miss\":" << p.counters.history_miss
              << ",\"fault_before_recovery\":" << fault_before
              << ",\"fresh_recovery\":" << p.Publishable()
              << ",\"max_published_sensor_gap_s\":" << max_gap
              << ",\"tail_silence_s\":" << tail_silence << "}";
}
void burst(bool incremental, double limit) {
    ImuOdomPredictor p(2, limit);
    p.Submit(correction(1, 1));
    feed(p, 1);
    unsigned publishable = 0;
    for (int i = 1; i <= 800; ++i) {
        p.Append(sample(1 + .005 * i));
        if (incremental) {
            p.Advance();
            publishable += p.Publishable();
        }
    }
    p.Advance();
    std::cout << "{\"incremental\":" << incremental << ",\"duration_s\":4,\"faulted\":" << p.Faulted()
              << ",\"final_publishable\":" << p.Publishable()
              << ",\"per_sample_publishable_checks\":" << publishable
              << ",\"history_miss\":" << p.counters.history_miss << "}";
}
}
int main(int argc, char** argv) {
    const double limit = argc > 1 ? std::atof(argv[1]) : .5;
    if (!(limit > 0)) return 2;
    std::cout << std::boolalpha << std::setprecision(12) << "{\"prediction_limit_s\":" << limit << ",\"delays\":[";
    delay(.75, limit); std::cout << ',';
    delay(1.5, limit); std::cout << ',';
    delay(2.5, limit); std::cout << "],\"bursts\":[";
    burst(false, limit); std::cout << ',';
    burst(true, limit); std::cout << "]}\n";
}
