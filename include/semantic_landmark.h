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

#include "ekf_management.h"
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

  virtual void AddSemanticMea(const double timestmap, const Pose& mea_pose,
                              const SemanticMea::Ptr mea) = 0;

  void NotifyAugmentation();

  void NotifyUpdate();

  void NotifyMarginalization();

  bool Initialized();

  bool Margin();

  virtual void InitializeLandmark(const Eigen::VectorXd& state,
                                  const Eigen::MatrixXd& P,
                                  Eigen::MatrixXd& Jx) = 0;

  virtual Eigen::VectorXd GetVectorizedData() = 0;

 protected:
  std::map<double, std::pair<Pose, SemanticMea::Ptr>> _meas;

 private:
  SensorType _type;
  int _id;
  bool _initialized;
  bool _margin;
};
}  // namespace apa_slam
