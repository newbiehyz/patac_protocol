/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/parking_slot_landmark.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 6:44:15 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <memory>

#include "local_mapping_define.h"
#include "semantic_landmark.h"

namespace apa_slam {
class ParkingSlotLandmark : public SemanticLandmark {
 public:
  typedef std::shared_ptr<ParkingSlotLandmark> Ptr;
  ParkingSlotLandmark(const int id, double* data);
  Eigen::MatrixXd GetLandmarkData();
  void AddSemanticMea(const long long timestmap, const Pose& mea_pose,
                      const SemanticMea::Ptr mea);

  void InitializeLandmark(const Eigen::VectorXd& state,
                          const Eigen::MatrixXd& P, Eigen::MatrixXd& Jx);

  Eigen::VectorXd GetVectorizedData();

  void GetResidualAndJacobian(const SemanticMea::Ptr& mea,
                              const Eigen::VectorXd& v_state,
                              Eigen::VectorXd& residual, Eigen::MatrixXd& J_v,
                              Eigen::MatrixXd& J_lm);

  void SetMean(const Eigen::VectorXd& mean);

  Eigen::VectorXd ComputeMatchingResidual(const Eigen::VectorXd& pose,
                                          const SemanticMea::Ptr& mea);

  Eigen::MatrixXd ConstructFullSlot();

 private:
  Eigen::MatrixXd _data;
};
}  // namespace apa_slam
