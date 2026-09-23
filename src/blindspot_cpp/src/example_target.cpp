// The example target the demos run against, and its calibration.
//
//     ros2 run blindspot_cpp example_target          # prints the path
//
// The guard ships no default threshold, on purpose: a threshold carries the
// target's scale and working distance. So the demos do what a real deployment
// does at commissioning time - calibrate on a KNOWN-GOOD target - using the
// six-point ring the whole study is built around, 0.05 m radius, 0.6 m
// standoff.
//
// Pass calibration:=/path.json to a launch file to use your own instead.
// Prints exactly one line, the path, so a launch file can read it back.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <Eigen/Core>

#include "blindspot/calibrate.hpp"
#include "blindspot/guard.hpp"

namespace
{

constexpr double kRingRadius = 0.05;
constexpr double kStandoff = 0.6;

Eigen::MatrixX3d ring(double r = kRingRadius, int n = 6)
{
  Eigen::MatrixX3d p(n, 3);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    p(i, 0) = r * std::cos(a);
    p(i, 1) = r * std::sin(a);
    p(i, 2) = 0.0;
  }
  return p;
}

}  // namespace

int main(int argc, char ** argv)
{
  // A path given on the command line is returned untouched, which is how the
  // launch files honour calibration:=/path.json.
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (!a.empty() && a[0] != '-') {
      std::cout << a << std::endl;
      return 0;
    }
  }
  try {
    Eigen::Matrix4d goal = Eigen::Matrix4d::Identity();
    goal(2, 3) = kStandoff;
    blindspot::CalibrateOptions opts;
    opts.n_poses = 4000;
    opts.percentile = 1.0;
    opts.seed = 0;
    const auto cal = blindspot::calibrate(ring(), goal, opts);

    const auto out =
      std::filesystem::temp_directory_path() / "blindspot_example_ring.json";
    std::ofstream f(out);
    if (!f) {
      throw std::runtime_error("cannot write " + out.string());
    }
    f << cal.to_json().dump(2) << "\n";
    f.close();
    std::cout << out.string() << std::endl;
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "example_target: " << e.what() << "\n";
    return 1;
  }
}
