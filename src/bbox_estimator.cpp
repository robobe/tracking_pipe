#include "bbox_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace tracking_pipe {
namespace {

constexpr std::int64_t kTickNs = 20'000'000;

double centre_x(const BBox& box) { return box.x + box.width / 2.0; }
double centre_y(const BBox& box) { return box.y + box.height / 2.0; }

double iou(const BBox& a, const BBox& b) {
  const double left = std::max(a.x, b.x);
  const double top = std::max(a.y, b.y);
  const double right = std::min(a.x + a.width, b.x + b.width);
  const double bottom = std::min(a.y + a.height, b.y + b.height);
  const double intersection = std::max(0.0, right - left) * std::max(0.0, bottom - top);
  const double area = a.width * a.height + b.width * b.height - intersection;
  return area > 0.0 ? intersection / area : 0.0;
}

int source_priority(Source source) { return source == Source::NanoTrack ? 0 : 1; }

bool valid_box(const BBox& box) {
  return std::isfinite(box.x) && std::isfinite(box.y) && std::isfinite(box.width) &&
         std::isfinite(box.height) && box.width > 0.0 && box.height > 0.0;
}

}  // namespace

Estimator::Estimator(Config config) : config_(std::move(config)) {}

void Estimator::submit(Measurement measurement) {
  pending_decision_sequences_.push_back(measurement.sequence);
  measurements_.push_back(std::move(measurement));
}

Estimate Estimator::tick(std::int64_t now_ns) {
  if (now_ns < 0 || (last_tick_ns_ >= 0 && now_ns <= last_tick_ns_)) {
    throw std::runtime_error("tick timestamps must be strictly increasing and non-negative");
  }

  const std::int64_t horizon_start = now_ns - config_.history_horizon_ns;
  const auto report_sequences = std::move(pending_decision_sequences_);
  pending_decision_sequences_.clear();
  std::vector<Decision> decisions;
  std::vector<Measurement> candidates;
  candidates.reserve(measurements_.size());
  for (const auto& measurement : measurements_) {
    if (!valid_box(measurement.bbox)) {
      decisions.push_back({measurement.sequence, false, "invalid_bbox"});
    } else if (measurement.confidence < 0.0 || measurement.confidence > 1.0) {
      decisions.push_back({measurement.sequence, false, "invalid_confidence"});
    } else if (measurement.capture_pts_ns > now_ns) {
      decisions.push_back({measurement.sequence, false, "future_measurement"});
    } else if (measurement.capture_pts_ns < horizon_start) {
      decisions.push_back({measurement.sequence, false, "outside_history"});
    } else {
      candidates.push_back(measurement);
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Measurement& a, const Measurement& b) {
    return std::tuple{a.capture_pts_ns, source_priority(a.source), a.sequence} <
           std::tuple{b.capture_pts_ns, source_priority(b.source), b.sequence};
  });

  if (!anchor_.has_value()) anchor_ = Snapshot{};
  State state = anchor_->state;
  const std::int64_t replay_start_ns = anchor_->time_ns;

  auto predict = [&state](std::int64_t target_ns) {
    if (!state.initialized || target_ns <= state.time_ns) return;
    const double seconds = static_cast<double>(target_ns - state.time_ns) / 1'000'000'000.0;
    state.box.x += state.vx * seconds;
    state.box.y += state.vy * seconds;
    state.time_ns = target_ns;
  };

  auto predicted_box = [&state](std::int64_t target_ns) {
    State copy = state;
    if (copy.initialized && target_ns > copy.time_ns) {
      const double seconds = static_cast<double>(target_ns - copy.time_ns) / 1'000'000'000.0;
      copy.box.x += copy.vx * seconds;
      copy.box.y += copy.vy * seconds;
    }
    return copy.box;
  };

  std::optional<BBox> retrack_roi;
  std::optional<std::int64_t> last_retrack_capture;
  std::optional<Measurement> nano_at_yolo_time;
  std::vector<Snapshot> rebuilt;
  std::int64_t next_snapshot_ns = replay_start_ns + kTickNs;

  for (const auto& measurement : candidates) {
    if (measurement.capture_pts_ns < replay_start_ns) continue;
    while (next_snapshot_ns < measurement.capture_pts_ns && next_snapshot_ns <= now_ns) {
      predict(next_snapshot_ns);
      rebuilt.push_back({next_snapshot_ns, state});
      next_snapshot_ns += kTickNs;
    }

    if (!state.initialized) {
      state.initialized = true;
      state.tracking = false;
      state.time_ns = measurement.capture_pts_ns;
      state.first_measurement_ns = measurement.capture_pts_ns;
      state.last_measurement_ns = measurement.capture_pts_ns;
      state.box = measurement.bbox;
      state.vx = 0.0;
      state.vy = 0.0;
      decisions.push_back({measurement.sequence, true, "seed"});
    } else {
      const BBox prediction = predicted_box(measurement.capture_pts_ns);
      const double scale = std::max({prediction.width, prediction.height, 1.0});
      const double distance = std::hypot(centre_x(prediction) - centre_x(measurement.bbox),
                                         centre_y(prediction) - centre_y(measurement.bbox));
      if (distance > scale * config_.association_distance_multiplier) {
        decisions.push_back({measurement.sequence, false, "association_gate"});
        continue;
      }
      predict(measurement.capture_pts_ns);
      const double seconds =
          static_cast<double>(measurement.capture_pts_ns - state.last_measurement_ns) / 1'000'000'000.0;
      const double gain = std::clamp(0.20 + 0.60 * measurement.confidence, 0.20, 0.80);
      const double dx = measurement.bbox.x - state.box.x;
      const double dy = measurement.bbox.y - state.box.y;
      state.box.x += gain * dx;
      state.box.y += gain * dy;
      state.box.width += gain * (measurement.bbox.width - state.box.width);
      state.box.height += gain * (measurement.bbox.height - state.box.height);
      if (seconds > 0.0) {
        state.vx += gain * (dx / seconds - state.vx);
        state.vy += gain * (dy / seconds - state.vy);
      }
      state.last_measurement_ns = measurement.capture_pts_ns;
      state.tracking = state.tracking || measurement.capture_pts_ns > state.first_measurement_ns;
      decisions.push_back({measurement.sequence, true, "update"});
    }

    if (measurement.source == Source::NanoTrack) {
      nano_at_yolo_time = measurement;
    } else if (nano_at_yolo_time.has_value() &&
               nano_at_yolo_time->capture_pts_ns == measurement.capture_pts_ns &&
               measurement.confidence >= config_.yolo_retrack_threshold &&
               nano_at_yolo_time->confidence < config_.nano_retrack_threshold &&
               iou(measurement.bbox, nano_at_yolo_time->bbox) < config_.retrack_iou_threshold &&
               (!last_retrack_capture.has_value() ||
                measurement.capture_pts_ns - *last_retrack_capture >= config_.retrack_cooldown_ns)) {
      last_retrack_capture = measurement.capture_pts_ns;
      if (!last_emitted_retrack_sequence_.has_value() ||
          *last_emitted_retrack_sequence_ != measurement.sequence) {
        retrack_roi = state.box;
        last_emitted_retrack_sequence_ = measurement.sequence;
      }
    }
  }

  while (next_snapshot_ns <= now_ns) {
    predict(next_snapshot_ns);
    rebuilt.push_back({next_snapshot_ns, state});
    next_snapshot_ns += kTickNs;
  }
  if (state.initialized) predict(now_ns);

  if (state.initialized && !state.tracking && now_ns - state.first_measurement_ns > config_.initialization_timeout_ns) {
    state = {};
  }

  snapshots_ = std::move(rebuilt);
  for (const auto& snapshot : snapshots_) {
    if (snapshot.time_ns < horizon_start) anchor_ = snapshot;
  }
  if (anchor_.has_value()) {
    measurements_.erase(
        std::remove_if(measurements_.begin(), measurements_.end(), [this](const Measurement& measurement) {
          return measurement.capture_pts_ns < anchor_->time_ns;
        }),
        measurements_.end());
  }
  last_tick_ns_ = now_ns;

  Estimate estimate;
  estimate.output_time_ns = now_ns;
  for (const auto& decision : decisions) {
    if (std::find(report_sequences.begin(), report_sequences.end(), decision.sequence) != report_sequences.end()) {
      estimate.decisions.push_back(decision);
    }
  }
  estimate.retrack_roi = retrack_roi;
  if (!state.initialized) return estimate;

  estimate.newest_capture_pts_ns = state.last_measurement_ns;
  estimate.bbox = state.box;
  estimate.velocity_x_px_s = state.vx;
  estimate.velocity_y_px_s = state.vy;
  const std::int64_t age = now_ns - state.last_measurement_ns;
  if (!state.tracking) {
    estimate.status = Status::Initializing;
  } else if (age == 0) {
    estimate.status = Status::Tracking;
    estimate.valid = true;
  } else {
    estimate.status = Status::Predicting;
    estimate.valid = age <= config_.prediction_limit_ns;
  }
  return estimate;
}

std::string to_string(Source source) {
  return source == Source::NanoTrack ? "nanotrack" : "yolo8";
}

std::string to_string(Status status) {
  switch (status) {
    case Status::NoTarget: return "NO_TARGET";
    case Status::Initializing: return "INITIALIZING";
    case Status::Tracking: return "TRACKING";
    case Status::Predicting: return "PREDICTING";
  }
  return "NO_TARGET";
}

Source source_from_string(const std::string& value) {
  if (value == "nanotrack") return Source::NanoTrack;
  if (value == "yolo8") return Source::Yolo8;
  throw std::runtime_error("source must be nanotrack or yolo8");
}

}  // namespace tracking_pipe
