#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tracking_pipe {

enum class Source { NanoTrack, Yolo8 };
enum class Status { NoTarget, Initializing, Tracking, Predicting };

struct BBox {
  double x = 0;
  double y = 0;
  double width = 0;
  double height = 0;
};

struct Measurement {
  std::uint64_t sequence = 0;
  std::int64_t capture_pts_ns = 0;
  Source source = Source::NanoTrack;
  BBox bbox;
  double confidence = 0;
  std::optional<int> class_id;
};

struct Config {
  std::int64_t history_horizon_ns = 500'000'000;
  std::int64_t prediction_limit_ns = 150'000'000;
  std::int64_t initialization_timeout_ns = 200'000'000;
  std::int64_t retrack_cooldown_ns = 500'000'000;
  double yolo_retrack_threshold = 0.80;
  double nano_retrack_threshold = 0.30;
  double retrack_iou_threshold = 0.30;
  double association_distance_multiplier = 4.0;
};

struct Decision {
  std::uint64_t sequence = 0;
  bool accepted = false;
  std::string reason;
};

struct Estimate {
  std::int64_t output_time_ns = 0;
  std::int64_t newest_capture_pts_ns = 0;
  BBox bbox;
  double velocity_x_px_s = 0;
  double velocity_y_px_s = 0;
  Status status = Status::NoTarget;
  bool valid = false;
  std::vector<Decision> decisions;
  std::optional<BBox> retrack_roi;
};

class Estimator {
 public:
  explicit Estimator(Config config = {});

  void submit(Measurement measurement);
 Estimate tick(std::int64_t now_ns);

 private:
  struct State {
    bool initialized = false;
    bool tracking = false;
    std::int64_t time_ns = 0;
    std::int64_t first_measurement_ns = 0;
    std::int64_t last_measurement_ns = 0;
    BBox box;
    double vx = 0.0;
    double vy = 0.0;
  };
  struct Snapshot {
    std::int64_t time_ns = 0;
    State state;
  };

  Config config_;
  std::vector<Measurement> measurements_;
  std::vector<Snapshot> snapshots_;
  std::optional<Snapshot> anchor_;
  std::int64_t last_tick_ns_ = -1;
  std::optional<std::uint64_t> last_emitted_retrack_sequence_;
  std::vector<std::uint64_t> pending_decision_sequences_;
};

std::string to_string(Source source);
std::string to_string(Status status);
Source source_from_string(const std::string& value);

}  // namespace tracking_pipe
