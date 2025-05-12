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
ParkingSlotLandmark::ParkingSlotLandmark(const int id, double *data)
    : SemanticLandmark(SEMANTIC_TYPE_PARKING_SLOT, id),
      _data(Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(
          data, DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT)) {}

Eigen::MatrixXd ParkingSlotLandmark::GetLandmarkData() { return _data; }

void ParkingSlotLandmark::AddSemanticMea(const double timestmap,
                                         const Pose &mea_pose,
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

  if (error_mea_swap.colwise().norm().norm() >
      error_mea.colwise().norm().norm()) {
    mea_data.col(0).swap(mea_data.col(1));
    mea->SetMeaData(mea_data);
  }

  _meas.insert({timestmap, {mea_pose, mea}});
}
}  // namespace apa_slam
