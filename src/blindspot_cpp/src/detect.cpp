#include "blindspot/detect.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace blindspot
{

namespace
{

struct Blob
{
  int area{0};
  double cx{0.0};
  double cy{0.0};
};

Eigen::MatrixX2d to_matrix(const std::vector<Blob> & blobs)
{
  Eigen::MatrixX2d out(static_cast<Eigen::Index>(blobs.size()), 2);
  for (size_t i = 0; i < blobs.size(); ++i) {
    out(static_cast<Eigen::Index>(i), 0) = blobs[i].cx;
    out(static_cast<Eigen::Index>(i), 1) = blobs[i].cy;
  }
  return out;
}

}  // namespace

Eigen::MatrixX2d detect_dots(const cv::Mat & rgb)
{
  cv::Mat gray;
  cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
  cv::Mat th;
  cv::threshold(gray, th, 90, 255, cv::THRESH_BINARY_INV);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int n = cv::connectedComponentsWithStats(th, labels, stats, centroids, 8);

  std::vector<Blob> cand;
  for (int i = 1; i < n; ++i) {            // label 0 is the background
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area > 25 && area < 4000) {
      cand.push_back(Blob{area, centroids.at<double>(i, 0), centroids.at<double>(i, 1)});
    }
  }
  if (cand.empty()) {
    return Eigen::MatrixX2d(0, 2);
  }
  if (cand.size() < 6) {
    return to_matrix(cand);
  }

  // The six dots are identical circles, so pick the six most SIMILAR blobs,
  // not the six largest. Taking the largest lets the robot's own dark wrist -
  // in frame at close standoff - masquerade as a fiducial and displace a real
  // dot. Sort by area, then slide a window of six and keep the one whose
  // largest-to-smallest ratio is tightest.
  std::stable_sort(
    cand.begin(), cand.end(), [](const Blob & a, const Blob & b) {return a.area < b.area;});

  size_t best_start = 0;
  double best_spread = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 6 <= cand.size(); ++i) {
    const double spread =
      static_cast<double>(cand[i + 5].area) / std::max(cand[i].area, 1);
    if (spread < best_spread) {
      best_start = i;
      best_spread = spread;
    }
  }
  const std::vector<Blob> best(cand.begin() + best_start, cand.begin() + best_start + 6);
  return to_matrix(best);
}

}  // namespace blindspot
