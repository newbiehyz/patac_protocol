/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/semantic_landmark.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Thursday, May
 * 8th 2025, 3:50:36 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <memory>

#include "local_mapping_define.h"
#include "semantic_measurement.h"
namespace apa_slam {
class SemanticLandmark {
 public:
  typedef std::shared_ptr<SemanticLandmark> Ptr;
  SemanticLandmark(const SensorType& type, const int id);
  virtual ~SemanticLandmark() = default;
  SensorType GetSemanticType();

  int GetId();
  virtual Eigen::MatrixXd GetLandmarkData() = 0;

  void AddSemanticMea(const double timestmap, const SemanticMea::Ptr mea);

  void InitializeLandmark();

  bool Initialized();

 private:
  SensorType _type;
  int _id;
  std::map<double, SemanticMea::Ptr> _meas;
  bool _initialized;
};
}  // namespace apa_slam
