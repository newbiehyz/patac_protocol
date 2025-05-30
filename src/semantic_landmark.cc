/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/semantic_landmark.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 6:53:08 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "semantic_landmark.h"

namespace apa_slam {
SemanticLandmark::SemanticLandmark(const SensorType& type, const int id)
    : _type(type),
      _id(id),
      _initialized(false),
      _margin(false),
      _update(false),
      _need_intialize(false) {}

SensorType SemanticLandmark::GetSemanticType() { return _type; }

int SemanticLandmark::GetId() { return _id; }

bool SemanticLandmark::Initialized() { return _initialized; }

bool SemanticLandmark::NeedMargin() { return _margin; }

void SemanticLandmark::SetMarginFlag(const bool& flag) { _margin = flag; }

bool SemanticLandmark::NeedUpdate() { return _update; }

void SemanticLandmark::SetUpdateFlag(const bool& flag) { _update = flag; }

void SemanticLandmark::SetNeedInitializeFlag(const bool& flag) {
  _need_intialize = flag;
}

bool SemanticLandmark::NeedInitialize() { return _need_intialize; }

void SemanticLandmark::SetInitializeFlag(const bool& flag) {
  _initialized = flag;
}

void SemanticLandmark::SetCov(const Eigen::MatrixXd& cov) { _cov = cov; }

void SemanticLandmark::TagMarginalization(const double timestamp) {
  if (ApaParameters::GetInstance().GetEstimatorParamters().time_scale *
          fabs(_meas.rbegin()->first - timestamp) >
      ApaParameters::GetInstance()
          .GetEstimatorParamters()
          .margin_tracking_time) {
    SetMarginFlag(true);
  }
}

Eigen::MatrixXd SemanticLandmark::GetCov() { return _cov; }

void SemanticLandmark::GetLatestResidualAndJacobian(
    const Eigen::VectorXd& v_state, Eigen::VectorXd& residual,
    Eigen::MatrixXd& J_v, Eigen::MatrixXd& J_lm) {
  auto latest_mea = _meas.rbegin()->second.second;
  std::cout << "mea time: " << std::to_string(_meas.rbegin()->first)
            << std::endl;
  GetResidualAndJacobian(latest_mea, v_state, residual, J_v, J_lm);
}

const SemanticMea::Ptr SemanticLandmark::GetLatestMea() {
  return _meas.rbegin()->second.second;
}

}  // namespace apa_slam
