#include "pcl_extractor.h"

namespace apa_slam {

PCLExtractor::PCLExtractor() {}

PCLExtractor& PCLExtractor::GetInstance() {
  static PCLExtractor instance;
  return instance;
}

Eigen::Vector3d PCLExtractor::uvToVehicle3D(const cv::Point& pt) {
  Eigen::Vector2d pt_uv(pt.x, pt.y);

  float REAR_AXEL_TO_CENTER =
      (APA_VEHICLE_LENGTH / 2) - APA_REAR_AXLE_CENTER_VEHICLE_REAR;

  float x = pt_uv.x() - SEG_VIEW_WIDTH / 2;
  float y = SEG_VIEW_HEIGHT / 2 - pt_uv.y() +
            (REAR_AXEL_TO_CENTER / APA_LR_BIRD_PIXECL_2_WORLD);

  Eigen::Vector3d corner_v;

  corner_v.x() = y * APA_LR_BIRD_PIXECL_2_WORLD;
  corner_v.y() = -x * APA_LR_BIRD_PIXECL_2_WORLD;
  corner_v.z() = 0;

  corner_v /= 1000.0f;

  return corner_v;
}

void PCLExtractor::quadtreeRecursive(const std::vector<Eigen::Vector2d>& points,
                                     double xmin, double xmax, double ymin,
                                     double ymax, int max_points,
                                     std::vector<Eigen::Vector2d>& out_points) {
  if (points.empty()) return;

  if (points.size() <= max_points) {
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
    out_points.push_back(closest_point);
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

  quadtreeRecursive(q1, xmin, xmid, ymin, ymid, max_points, out_points);
  quadtreeRecursive(q2, xmid, xmax, ymin, ymid, max_points, out_points);
  quadtreeRecursive(q3, xmin, xmid, ymid, ymax, max_points, out_points);
  quadtreeRecursive(q4, xmid, xmax, ymid, ymax, max_points, out_points);
}

void PCLExtractor::extractAllClassesQuadtree(const cv::Mat& seg,
                                             int max_points_per_cell) {
  if (seg.empty() || seg.channels() != 1) return;

  std::map<SemanticClass, std::vector<Eigen::Vector2d>> class_points_2d;

  for (int r = 0; r < seg.rows; ++r) {
    const uchar* ptr = seg.ptr<uchar>(r);
    for (int c = 0; c < seg.cols; ++c) {
      uchar val = ptr[c];
      if (val < static_cast<uint8_t>(SemanticClass::LaneSolid) ||
          val > static_cast<uint8_t>(SemanticClass::ArrowLeftRightFront))
        continue;

      Eigen::Vector3d pt3d = uvToVehicle3D(cv::Point(c, r));
      class_points_2d[static_cast<SemanticClass>(val)].emplace_back(pt3d.x(),
                                                                    pt3d.y());
    }
  }

  for (auto& [cls, points] : class_points_2d) {
    if (points.empty()) continue;

    double xmin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymin = std::numeric_limits<double>::max();
    double ymax = std::numeric_limits<double>::lowest();

    for (const auto& p : points) {
      if (p.x() < xmin) xmin = p.x();
      if (p.x() > xmax) xmax = p.x();
      if (p.y() < ymin) ymin = p.y();
      if (p.y() > ymax) ymax = p.y();
    }

    std::vector<Eigen::Vector2d> sampled_points;

    quadtreeRecursive(points, xmin, xmax, ymin, ymax, max_points_per_cell,
                      sampled_points);

    for (const auto& p : sampled_points)
      class_points_[cls].emplace_back(p.x(), p.y(), 0.0);
  }
}

const std::map<SemanticClass, std::vector<Eigen::Vector3d>>&
PCLExtractor::GetAllPoints() const {
  return class_points_;
}

void PCLExtractor::ClearAllPoints() { class_points_.clear(); }

}  // namespace apa_slam