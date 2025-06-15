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
#include "action_queue.h"
#include "apa_parameters.h"
#include "ekf_estimator.h"
#include "fillback_data_loader.h"
#include "kinematic_measurement.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "semantic_measurement.h"
namespace apa_slam {
class LocalMappingInterface {
 public:
  LocalMappingInterface();
  static LocalMappingInterface& GetInstance();
  void Reset();
  void Init(const std::string& cfg_json);
  void ProcDrPose(long long timestamp, const Eigen::VectorXd& pose);
  void ProcSlotData(long long timestamp_d,
                    const std::vector<Eigen::VectorXd>& slot_data,
                    const std::vector<ParkingSlotAttribute>& slot_attribute);
  bool GetLatestVehiclePose(Eigen::VectorXd& pose);

  bool GetLatestSlotMap(std::map<int, Eigen::MatrixXd>& slot_map,
                        std::map<int, ParkingSlotAttribute>& slot_attri);

  void SetTargetSlotId(const int id);
 private:
  std::string _output_file_name;
  std::string _cfg;
};

}  // namespace apa_slam
