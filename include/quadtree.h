#pragma once
#include <Eigen/Eigen>
#include <vector>

namespace apa_slam {

class Quadtree {
 public:
  Quadtree(int max_points_per_cell);

  void Build(const std::vector<Eigen::Vector2d>& points, double xmin,
             double xmax, double ymin, double ymax);

  const std::vector<Eigen::Vector2d>& GetSampledPoints() const;

 private:
  void quadtreeRecursive(const std::vector<Eigen::Vector2d>& points,
                         double xmin, double xmax, double ymin, double ymax);

  int max_points_;
  std::vector<Eigen::Vector2d> sampled_points_;
};

}  // namespace apa_slam
