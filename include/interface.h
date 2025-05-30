/*
 * Filename:
 * /home/yukan/Documents/work/U557_Parking_J5/include/apa_local_mapping/interface.h
 * Path: /home/yukan/Documents/work/U557_Parking_J5/include/apa_local_mapping
 * Created Date: Saturday, May 24th 2025, 3:21:06 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <memory>

#include "apa_parameters.h"
#include "ekf_estimator.h"
#include "kinematic_measurement.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "semantic_measurement.h"

#include "fillback_data_loader.h"
namespace apa_slam {
class LocalMappingInterface {
 public:
  LocalMappingInterface();
  static LocalMappingInterface& GetInstance();
  void Reset();
  void Init(const std::string& cfg_json);
  void ProcDrPose(double timestamp_d, const Eigen::VectorXd& pose);
  void ProcSlotData(double timestamp_d,
                    const std::vector<Eigen::VectorXd>& slot_data);
  bool GetLatestVehiclePose(Eigen::VectorXd& pose);

 private:
};

}  // namespace apa_slam
