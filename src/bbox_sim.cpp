#include "bbox_estimator.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

using json = nlohmann::json;
using tracking_pipe::BBox;
using tracking_pipe::Estimate;
using tracking_pipe::Measurement;

struct ScheduledMeasurement {
  std::int64_t arrival_ns = 0;
  Measurement measurement;
};

struct Scenario {
  std::vector<std::int64_t> ticks;
  std::vector<ScheduledMeasurement> measurements;
};

std::int64_t required_i64(const json& value, const char* key) {
  if (!value.contains(key) || !value.at(key).is_number_integer()) {
    throw std::runtime_error(std::string("missing integer ") + key);
  }
  return value.at(key).get<std::int64_t>();
}

double required_double(const json& value, const char* key) {
  if (!value.contains(key) || !value.at(key).is_number()) {
    throw std::runtime_error(std::string("missing number ") + key);
  }
  return value.at(key).get<double>();
}

BBox read_box(const json& value) {
  const auto& array = value.at("bbox_xywh_px");
  if (!array.is_array() || array.size() != 4) throw std::runtime_error("bbox_xywh_px must contain four numbers");
  return {array.at(0).get<double>(), array.at(1).get<double>(), array.at(2).get<double>(), array.at(3).get<double>()};
}

Scenario read_scenario(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path);
  Scenario scenario;
  std::string line;
  std::uint64_t sequence = 0;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty()) continue;
    try {
      const auto record = json::parse(line);
      const std::string type = record.at("type").get<std::string>();
      if (type == "tick") {
        scenario.ticks.push_back(required_i64(record, "time_ns"));
      } else if (type == "measurement") {
        Measurement measurement;
        measurement.sequence = ++sequence;
        measurement.capture_pts_ns = required_i64(record, "capture_pts_ns");
        measurement.source = tracking_pipe::source_from_string(record.at("source").get<std::string>());
        measurement.bbox = read_box(record);
        measurement.confidence = required_double(record, "confidence");
        if (record.contains("class_id")) measurement.class_id = record.at("class_id").get<int>();
        scenario.measurements.push_back({required_i64(record, "arrival_ns"), measurement});
      } else {
        throw std::runtime_error("type must be tick or measurement");
      }
    } catch (const std::exception& error) {
      throw std::runtime_error(path + ":" + std::to_string(line_number) + ": " + error.what());
    }
  }
  if (scenario.ticks.empty()) throw std::runtime_error("scenario has no tick records");
  std::sort(scenario.ticks.begin(), scenario.ticks.end());
  if (std::adjacent_find(scenario.ticks.begin(), scenario.ticks.end()) != scenario.ticks.end()) {
    throw std::runtime_error("tick records must have unique time_ns values");
  }
  std::sort(scenario.measurements.begin(), scenario.measurements.end(), [](const auto& a, const auto& b) {
    return std::tie(a.arrival_ns, a.measurement.sequence) < std::tie(b.arrival_ns, b.measurement.sequence);
  });
  return scenario;
}

json to_json(const Estimate& estimate) {
  json value = {
      {"output_time_ns", estimate.output_time_ns},
      {"newest_capture_pts_ns", estimate.newest_capture_pts_ns},
      {"bbox_xywh_px", {estimate.bbox.x, estimate.bbox.y, estimate.bbox.width, estimate.bbox.height}},
      {"velocity_xy_px_s", {estimate.velocity_x_px_s, estimate.velocity_y_px_s}},
      {"status", tracking_pipe::to_string(estimate.status)},
      {"valid", estimate.valid},
      {"retrack_roi_xywh_px", nullptr},
      {"decisions", json::array()},
  };
  if (estimate.retrack_roi.has_value()) {
    const auto& box = *estimate.retrack_roi;
    value["retrack_roi_xywh_px"] = {box.x, box.y, box.width, box.height};
  }
  for (const auto& decision : estimate.decisions) {
    value["decisions"].push_back({{"sequence", decision.sequence}, {"accepted", decision.accepted}, {"reason", decision.reason}});
  }
  return value;
}

bool equal_json(const json& expected, const json& actual, const std::string& path, std::string& error) {
  constexpr double kTolerance = 1e-6;
  if (expected.is_number() && actual.is_number()) {
    if (std::abs(expected.get<double>() - actual.get<double>()) <= kTolerance) return true;
    error = path + ": expected " + expected.dump() + ", got " + actual.dump();
    return false;
  }
  if (expected.type() != actual.type()) {
    error = path + ": JSON types differ";
    return false;
  }
  if (expected.is_array()) {
    if (expected.size() != actual.size()) {
      error = path + ": array sizes differ";
      return false;
    }
    for (std::size_t index = 0; index < expected.size(); ++index) {
      if (!equal_json(expected.at(index), actual.at(index), path + "[" + std::to_string(index) + "]", error)) return false;
    }
    return true;
  }
  if (expected.is_object()) {
    if (expected.size() != actual.size()) {
      error = path + ": object sizes differ";
      return false;
    }
    for (auto iterator = expected.begin(); iterator != expected.end(); ++iterator) {
      if (!actual.contains(iterator.key())) {
        error = path + ": missing key " + iterator.key();
        return false;
      }
      if (!equal_json(iterator.value(), actual.at(iterator.key()), path + "." + iterator.key(), error)) return false;
    }
    return true;
  }
  if (expected != actual) {
    error = path + ": expected " + expected.dump() + ", got " + actual.dump();
    return false;
  }
  return true;
}

std::vector<json> read_jsonl(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path);
  std::vector<json> records;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty()) records.push_back(json::parse(line));
  }
  return records;
}

void usage() { std::cerr << "usage: bbox_sim SCENARIO.jsonl [--expected EXPECTED.jsonl] [--trace]\n"; }

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2) {
      usage();
      return 2;
    }
    std::string scenario_path = argv[1];
    std::string expected_path;
    bool trace = false;
    for (int index = 2; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--trace") {
        trace = true;
      } else if (argument == "--expected" && index + 1 < argc) {
        expected_path = argv[++index];
      } else {
        usage();
        return 2;
      }
    }

    const Scenario scenario = read_scenario(scenario_path);
    tracking_pipe::Estimator estimator;
    std::vector<json> output;
    std::size_t next_measurement = 0;
    for (const auto tick : scenario.ticks) {
      while (next_measurement < scenario.measurements.size() &&
             scenario.measurements[next_measurement].arrival_ns <= tick) {
        estimator.submit(scenario.measurements[next_measurement++].measurement);
      }
      const auto result = to_json(estimator.tick(tick));
      if (trace) {
        std::cerr << tick << " " << result.at("status") << " " << result.at("bbox_xywh_px").dump() << '\n';
      }
      output.push_back(result);
      std::cout << result.dump() << '\n';
    }
    if (next_measurement != scenario.measurements.size()) {
      throw std::runtime_error("scenario contains measurements arriving after its final tick");
    }
    if (!expected_path.empty()) {
      const auto expected = read_jsonl(expected_path);
      if (expected.size() != output.size()) {
        throw std::runtime_error("expected output count differs from simulation output");
      }
      for (std::size_t index = 0; index < expected.size(); ++index) {
        std::string error;
        if (!equal_json(expected[index], output[index], "line " + std::to_string(index + 1), error)) {
          throw std::runtime_error(error);
        }
      }
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "bbox_sim: " << error.what() << '\n';
    return 1;
  }
}
