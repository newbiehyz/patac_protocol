/*
 * Filename:
 * /home/yukan/Documents/work/U557_Parking_J5/src/apa_local_mapping/interface.cc
 * Path: /home/yukan/Documents/work/U557_Parking_J5/src/apa_local_mapping
 * Created Date: Saturday, May 24th 2025, 3:20:51 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "interface.h"

namespace apa_slam {
LocalMappingInterface::LocalMappingInterface() {}

LocalMappingInterface &LocalMappingInterface::GetInstance() {
  static LocalMappingInterface instance;
  return instance;
}

void LocalMappingInterface::Init(const std::string &cfg_json) {
  ApaParameters::GetInstance().LoadParameters(cfg_json);
  EkfEstimator::GetInstance().Init();
}

void LocalMappingInterface::ProcDrPose(double timestamp_d,
                                       const Eigen::VectorXd &pose) {
  apa_slam::Pose p;
  p.x = pose.x();
  p.y = pose.y();
  p.yaw = pose.z();
  double v_out, w_out, ts_out;
  if (EkfEstimator::GetInstance().ProcDrPose(timestamp_d, p, ts_out, v_out,
                                             w_out)) {
    Eigen::VectorXd mea_data = Eigen::VectorXd::Zero(2);
    mea_data[0] = v_out;
    mea_data[1] = w_out;

    KinematicMea::Ptr odo_mea =
        std::make_shared<OdoMea>(ts_out, mea_data.data());
    std::vector<KinematicMea::Ptr> mea_vector;
    mea_vector.push_back(odo_mea);
    EkfEstimator::GetInstance().InputKinematicMea(ts_out, mea_vector);
  }
}

bool LocalMappingInterface::GetLatestVehiclePose(Eigen::VectorXd &pose) {
  if (!EkfEstimator::GetInstance().Initialized()) {
    return false;
  }

  auto vehicle_pose =EkfEstimator::GetInstance().GetLatestPose();
  pose = Eigen::VectorXd::Zero(3);
  pose[0] = vehicle_pose.x;
  pose[1] = vehicle_pose.y;
  pose[2] = vehicle_pose.yaw;

  return true;
}

}  // namespace apa_slam
