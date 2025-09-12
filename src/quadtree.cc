#include "quadtree.h"
#include <limits>

namespace apa_slam {

Quadtree::Quadtree(int max_points_per_cell)
    : max_points_(max_points_per_cell) {}

void Quadtree::Build(const std::vector<Eigen::Vector2d>& points,
                     double xmin, double xmax,
                     double ymin, double ymax) {
  sampled_points_.clear();
  quadtreeRecursive(points, xmin, xmax, ymin, ymax);
}

const std::vector<Eigen::Vector2d>& Quadtree::GetSampledPoints() const {
  return sampled_points_;
}

void Quadtree::quadtreeRecursive(const std::vector<Eigen::Vector2d>& points,
                                 double xmin, double xmax,
                                 double ymin, double ymax) {
  if (points.empty()) return;

  if (points.size() <= max_points_) {
    // 取离中心最近的点
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (auto& p : points) center += p;
    center /= points.size();

    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d closest_point;
    for (auto& p : points) {
      double d = (p - center).squaredNorm();
      if (d < min_dist) {
        min_dist = d;
        closest_point = p;
      }
    }
    sampled_points_.push_back(closest_point);
    return;
  }

  double xmid = (xmin + xmax) * 0.5;
  double ymid = (ymin + ymax) * 0.5;

  std::vector<Eigen::Vector2d> q1, q2, q3, q4;
  for (const auto& p : points) {
    if (p.x() <= xmid && p.y() <= ymid)
      q1.push_back(p);
    else if (p.x() > xmid && p.y() <= ymid)
      q2.push_back(p);
    else if (p.x() <= xmid && p.y() > ymid)
      q3.push_back(p);
    else
      q4.push_back(p);
  }

  quadtreeRecursive(q1, xmin, xmid, ymin, ymid);
  quadtreeRecursive(q2, xmid, xmax, ymin, ymid);
  quadtreeRecursive(q3, xmin, xmid, ymid, ymax);
  quadtreeRecursive(q4, xmid, xmax, ymid, ymax);
}

}  // namespace apa_slam