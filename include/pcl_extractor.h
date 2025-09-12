#pragma once
#include <Eigen/Eigen>
#include <map>
#include <opencv2/opencv.hpp>
#include <vector>
#include "fillback_data_loader.h"
#include "quadtree.h"

namespace apa_slam {

enum SemanticClass : uint8_t {
  LaneSolid = 1,
  LaneDashed = 2,
  ParkingLine = 3,
  Zebra = 4,
  SpeedBump = 5,
  StopLine = 6,
  NoParkingZone = 7,
  ArrowLeft = 8,
  ArrowRight = 9,
  ArrowLeftFront = 10,
  ArrowRightFront = 11,
  ArrowLeftRight = 12,
  ArrowStraight = 13,
  ArrowTurn = 14,
  ArrowLeftRightFront = 15
};

class PCLExtractor {
 public:
  PCLExtractor();
  static PCLExtractor& GetInstance();

  void extractAllClassesQuadtree(const cv::Mat& seg, int max_points_per_cell);

  const std::map<SemanticClass, std::vector<Eigen::Vector3d>>& GetAllPoints()
      const;
  void ClearAllPoints();

 private:
  std::map<SemanticClass, std::vector<Eigen::Vector3d>> class_points_;

  Eigen::Vector3d uvToVehicle3D(const cv::Point& pt);
};

}  // namespace apa_slam