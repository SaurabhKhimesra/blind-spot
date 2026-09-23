// Calibrate guard thresholds on a KNOWN-GOOD target. See calibrate.hpp.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "blindspot/calibrate.hpp"
#include "blindspot/guard.hpp"

namespace
{

void usage()
{
  std::cerr <<
    "usage: ros2 run blindspot_cpp calibrate --geometry target.json \\\n"
    "           --goal-pose pose.json -o my_target.json\n"
    "           [--poses 4000] [--percentile 1.0] [--seed 0]\n"
    "\n"
    "--geometry  JSON {\"points\": [[x,y,z], ...]}, metres\n"
    "--goal-pose JSON {\"cTo\": 4x4} or {\"R\": 3x3, \"t\": [x,y,z]}\n";
}

nlohmann::json read_json(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open " + path);
  }
  nlohmann::json j;
  f >> j;
  return j;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string geometry;
  std::string goal_pose;
  std::string out;
  blindspot::CalibrateOptions opts;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error(a + " needs a value");
        }
        return argv[++i];
      };
    if (a == "--geometry") {
      geometry = next();
    } else if (a == "--goal-pose") {
      goal_pose = next();
    } else if (a == "-o" || a == "--out") {
      out = next();
    } else if (a == "--poses") {
      opts.n_poses = std::stoi(next());
    } else if (a == "--percentile") {
      opts.percentile = std::stod(next());
    } else if (a == "--seed") {
      opts.seed = static_cast<unsigned int>(std::stoul(next()));
    } else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else if (a.rfind("--ros-args", 0) == 0) {
      break;                                // ignore anything ros2 run appends
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      usage();
      return 2;
    }
  }
  if (geometry.empty() || goal_pose.empty() || out.empty()) {
    usage();
    return 2;
  }

  try {
    const auto geo = read_json(geometry);
    const auto & pts = geo.at("points");
    Eigen::MatrixX3d points(static_cast<Eigen::Index>(pts.size()), 3);
    for (size_t i = 0; i < pts.size(); ++i) {
      for (int c = 0; c < 3; ++c) {
        points(static_cast<Eigen::Index>(i), c) = pts.at(i).at(c).get<double>();
      }
    }
    const Eigen::Matrix4d goal = blindspot::load_pose(read_json(goal_pose));
    const auto cal = blindspot::calibrate(points, goal, opts);

    std::ofstream f(out);
    if (!f) {
      throw std::runtime_error("cannot write " + out);
    }
    f << cal.to_json().dump(2) << "\n";
    f.close();

    std::printf("wrote %s (%s)\n", out.c_str(), blindspot::kCalibrationSchema);
    std::printf("thresholds by visible feature count:\n");
    for (const auto & [n, v] : cal.thresholds()) {
      std::printf("   N=%-3d %.6e\n", n, v);
    }
    std::printf(
      "\nThis is only valid if the target you calibrated on was healthy.\n");
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "calibrate: " << e.what() << "\n";
    return 1;
  }
}
