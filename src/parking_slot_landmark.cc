/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/parking_slot_landmark.cc Path:
 * /home/yukan/Documents/work/local_mapping/src Created Date: Thursday, May 8th
 * 2025, 6:58:35 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "parking_slot_landmark.h"

namespace apa_slam {
ParkingSlotLandmark::ParkingSlotLandmark(const int id, double* data)
    : SemanticLandmark(SEMANTIC_TYPE_PARKING_SLOT, id),
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(
          data, DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT)) {}

Eigen::MatrixXd ParkingSlotLandmark::GetLandmarkData() { return _data; }

void ParkingSlotLandmark::AddSemanticMea(const long long timestmap,
                                         const Pose& mea_pose,
                                         const SemanticMea::Ptr mea) {
  Eigen::Matrix2d mea_data = mea->GetMeaData().topLeftCorner(2, 2);
  Eigen::Vector2d twb(mea_pose.x, mea_pose.y);
  Eigen::Rotation2Dd rot(mea_pose.yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  Eigen::Matrix2d pt_w = Rwb * mea_data + twb.replicate(1, 2);

  Eigen::Matrix2d pt_w_swap = pt_w;
  pt_w_swap.col(0).swap(pt_w_swap.col(1));

  bool swap_mea_cols = false;
  Eigen::Matrix2d error_mea = _data - pt_w;
  Eigen::Matrix2d error_mea_swap = _data - pt_w_swap;

  if (error_mea_swap.colwise().norm().norm() <
      error_mea.colwise().norm().norm()) {
    mea_data.col(0).swap(mea_data.col(1));
    mea->SetMeaData(mea_data);
  }

  _meas.insert({timestmap, {mea_pose, mea}});

  // if (_meas.size() >= ApaParameters::GetInstance()
  //                         .GetEstimatorParamters()
  //                         .slot_min_tracking_times &&
  //     !Initialized()) {
  //   SetNeedInitializeFlag(true);
  // }

  // if (_meas.size() > ApaParameters::GetInstance()
  //                        .GetEstimatorParamters()
  //                        .slot_min_tracking_times &&
  //     !NeedInitialize() && Initialized()) {
  //   SetUpdateFlag(true);
  // }
}

void ParkingSlotLandmark::InitializeLandmark(const long long timestamp,
                                             const Eigen::VectorXd& state,
                                             const Eigen::MatrixXd& P,
                                             Eigen::MatrixXd& Jx) {
  Eigen::Vector2d twb = state.head(2);
  double yaw = state[2];
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  // Rwb << cos, -sin,
  //        sin, cos

  auto mea = _meas.at(timestamp).second;
  auto slot_mea = std::dynamic_pointer_cast<ParkingSlotMea>(mea);

  Eigen::Matrix2d mea_data = mea->GetMeaData().topLeftCorner(2, 2);
  Eigen::Matrix2d pt_w = Rwb * mea_data + twb.replicate(1, 2);

  _data = Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT);
  _data.topLeftCorner(2, 2) = pt_w;

  Jx = Eigen::MatrixXd::Zero(STATE_PARKING_SLOT_SIZE, STATE_VEHICLE_SIZE);
  Jx.block(0, 0, 2, 2).setIdentity();
  Jx.block(2, 0, 2, 2).setIdentity();

  Eigen::Matrix2d d_Rwb_theta;
  d_Rwb_theta << -std::sin(yaw), -std::cos(yaw), std::cos(yaw), -std::sin(yaw);

  Jx.block(0, 2, 2, 1) = d_Rwb_theta * mea_data.col(0);
  Jx.block(2, 2, 2, 1) = d_Rwb_theta * mea_data.col(1);

  Eigen::MatrixXd J0 = Jx.topRows(2);
  Eigen::MatrixXd J1 = Jx.bottomRows(2);

  Eigen::MatrixXd mea_cov =
      _meas.rbegin()->second.second->GetMeasurementNosise().topLeftCorner(2, 2);
  Eigen::MatrixXd cov =
      Eigen::MatrixXd::Zero(STATE_PARKING_SLOT_SIZE, STATE_PARKING_SLOT_SIZE);
  cov.topLeftCorner(2, 2) =
      J0 * P * J0.transpose() + Rwb * mea_cov * Rwb.transpose();
  cov.bottomRightCorner(2, 2) =
      J1 * P * J1.transpose() + Rwb * mea_cov * Rwb.transpose();

  SetCov(cov);

  SetInitializeFlag(true);

  SetAttribute(slot_mea->GetAttribute());
}

void ParkingSlotLandmark::InitializeLandmark(const Eigen::VectorXd& state,
                                             const Eigen::MatrixXd& P,
                                             Eigen::MatrixXd& Jx) {
  Eigen::Vector2d twb = state.head(2);
  double yaw = state[2];
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();
  // Rwb << cos, -sin,
  //        sin, cos

  // use last mea to initialize
  Eigen::Matrix2d last_mea_data =
      _meas.rbegin()->second.second->GetMeaData().topLeftCorner(2, 2);
  Eigen::Matrix2d pt_w = Rwb * last_mea_data + twb.replicate(1, 2);

  _data = Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT);
  _data.topLeftCorner(2, 2) = pt_w;

  Jx = Eigen::MatrixXd::Zero(STATE_PARKING_SLOT_SIZE, STATE_VEHICLE_SIZE);
  Jx.block(0, 0, 2, 2).setIdentity();
  Jx.block(2, 0, 2, 2).setIdentity();

  Eigen::Matrix2d d_Rwb_theta;
  d_Rwb_theta << -std::sin(yaw), -std::cos(yaw), std::cos(yaw), -std::sin(yaw);

  Jx.block(0, 2, 2, 1) = d_Rwb_theta * last_mea_data.col(0);
  Jx.block(2, 2, 2, 1) = d_Rwb_theta * last_mea_data.col(1);

  Eigen::MatrixXd J0 = Jx.topRows(2);
  Eigen::MatrixXd J1 = Jx.bottomRows(2);

  Eigen::MatrixXd mea_cov =
      _meas.rbegin()->second.second->GetMeasurementNosise().topLeftCorner(2, 2);
  Eigen::MatrixXd cov =
      Eigen::MatrixXd::Zero(STATE_PARKING_SLOT_SIZE, STATE_PARKING_SLOT_SIZE);
  cov.topLeftCorner(2, 2) =
      J0 * P * J0.transpose() + Rwb * mea_cov * Rwb.transpose();
  cov.bottomRightCorner(2, 2) =
      J1 * P * J1.transpose() + Rwb * mea_cov * Rwb.transpose();

  SetCov(cov);

  SetInitializeFlag(true);
  SetNeedInitializeFlag(false);
}

Eigen::VectorXd ParkingSlotLandmark::GetVectorizedData() {
  Eigen::VectorXd vector_data = Eigen::VectorXd::Zero(STATE_PARKING_SLOT_SIZE);
  vector_data.head(2) = _data.col(0).head(2);
  vector_data.tail(2) = _data.col(1).head(2);

  return vector_data;
}

void ParkingSlotLandmark::SetMean(const Eigen::VectorXd& mean) {
  _data.col(0).head(2) = mean.head(2);
  _data.col(1).head(2) = mean.tail(2);
}

Eigen::MatrixXd ParkingSlotLandmark::ConstructFullSlot() {
  Eigen::Vector2d dir =
      (_data.col(0).head(2) - _data.col(1).head(2)).normalized();
  Eigen::Vector2d dir_inner(-dir.y(), dir.x());
  Eigen::MatrixXd data_slot = Eigen::MatrixXd::Zero(2, 4);
  data_slot.leftCols(2) = _data;
  data_slot.col(2) =
      _data.col(1).head(2) +
      dir_inner * ApaParameters::GetInstance().GetEstimatorParamters().slot_len;
  data_slot.col(3) =
      _data.col(0).head(2) +
      dir_inner * ApaParameters::GetInstance().GetEstimatorParamters().slot_len;

  return data_slot;
}

void ParkingSlotLandmark::SetAttribute(const ParkingSlotAttribute& attri) {
  _attri = attri;
}

ParkingSlotAttribute ParkingSlotLandmark::GetAttribute() { return _attri; }

void ParkingSlotLandmark::SetTarget() { _target = true; }

bool ParkingSlotLandmark::IsTarget() { return _target; }

Eigen::VectorXd ParkingSlotLandmark::ComputeMatchingResidual(
    const Eigen::VectorXd& pose, const SemanticMea::Ptr& mea) {
  Eigen::VectorXd matching_residual = Eigen::VectorXd::Zero(2);

  Eigen::MatrixXd lm_data = this->GetLandmarkData();
  Eigen::Vector2d twb(pose.x(), pose.y());
  Eigen::Rotation2Dd rot(pose.z());
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();

  Eigen::Matrix2d pt_w =
      Rwb * mea->GetMeaData().topLeftCorner(2, 2) + twb.replicate(1, 2);

  matching_residual[0] =
      ((pt_w.col(0) + pt_w.col(1) - lm_data.col(0) - lm_data.col(1)) * .5)
          .norm();

  Eigen::Vector2d dir0 = (pt_w.col(0) - pt_w.col(1)).normalized();
  Eigen::Vector2d dir1 =
      (lm_data.col(0).head(2) - lm_data.col(1).head(2)).normalized();

  double angular_distance = std::acos(fabs(dir0.dot(dir1)));

  matching_residual[1] = angular_distance;  // ra
  return matching_residual;
}

void ParkingSlotLandmark::GetResidualAndJacobian(const SemanticMea::Ptr& mea,
                                                 const Eigen::VectorXd& v_state,
                                                 Eigen::VectorXd& residual,
                                                 Eigen::MatrixXd& J_v,
                                                 Eigen::MatrixXd& J_lm) {
  Eigen::Vector2d twb = v_state.head(2);
  double yaw = v_state[2];
  Eigen::Rotation2Dd rot(yaw);
  Eigen::Matrix2d Rwb = rot.toRotationMatrix();

  // residual = observation - prediciton
  residual = Eigen::VectorXd::Zero(RESIDUAL_PARKING_SLOT_SIZE);
  Eigen::VectorXd observation = mea->GetVectorizedData();
  Eigen::Matrix2d Rbw = Rwb.transpose();
  Eigen::Vector2d tbw = -Rwb.transpose() * twb;

  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R.topLeftCorner(2, 2) = Rbw;
  R.bottomRightCorner(2, 2) = Rbw;

  Eigen::Vector4d t = tbw.replicate(2, 1);
  residual = R * this->GetVectorizedData() + t - observation;
  Eigen::VectorXd lm = this->GetVectorizedData();
  lm.head(2).swap(lm.tail(2));
  // auto r = observation - R * lm + t;
  // std::cout << "rrr: " << std::to_string(this->GetId()) << " "
  // << residual.transpose() << std::endl;

  J_lm = Eigen::MatrixXd::Zero(RESIDUAL_PARKING_SLOT_SIZE,
                               STATE_PARKING_SLOT_SIZE);
  J_v = Eigen::MatrixXd::Zero(RESIDUAL_PARKING_SLOT_SIZE, STATE_VEHICLE_SIZE);

  J_lm.topLeftCorner(2, 2) = -Rbw;
  J_lm.bottomRightCorner(2, 2) = -Rbw;

  Eigen::Matrix2d d_Rwb_d_yaw;
  d_Rwb_d_yaw << -sin(yaw), -cos(yaw), cos(yaw), -sin(yaw);

  J_v.topLeftCorner(2, 2) = Rwb.transpose();
  J_v.bottomLeftCorner(2, 2) = Rwb.transpose();
  J_v.topRightCorner(2, 1) =
      -d_Rwb_d_yaw.transpose() * this->GetLandmarkData().col(0).head(2) +
      d_Rwb_d_yaw.transpose() * twb;
  J_v.bottomRightCorner(2, 1) =
      -d_Rwb_d_yaw.transpose() * this->GetLandmarkData().col(1).head(2) +
      d_Rwb_d_yaw.transpose() * twb;
}
}  // namespace apa_slam
