/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/sim_data_loader.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 1:10:50 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <Eigen/Eigen>
#include <fstream>
#include <memory>

#include "apa_parameters.h"
#include "kinematic_measurement.h"
#include "local_mapping_define.h"
#include "odo_measurement.h"
#include "parking_slot_landmark.h"
#include "parking_slot_measurement.h"
#include "semantic_landmark.h"
#include "semantic_map.h"
#include "semantic_measurement.h"

namespace apa_slam {

class SimDataLoader {
 public:
  typedef std::shared_ptr<SimDataLoader> Ptr;
  SimDataLoader();
  void LoadDataSet(const std::string& dataset_path);
  bool PopOutMea(ReplaySensorType& type, double& ts);
  std::vector<SemanticMea::Ptr> GetSemanticMeas(const double timestamp);
  std::vector<KinematicMea::Ptr> GetKinematicMeas(const double timestamp);

 private:
  void load_dataset_pose(const std::string& pose_file);
  void load_dataset_map(const std::string& map_file);
  void generate_semantic_meas();

  std::map<double, Pose> _gt_pose;
  std::map<double, std::vector<SemanticMea::Ptr>> _semantic_mea;
  std::map<double, std::vector<KinematicMea::Ptr>> _kinematic_mea;
  std::map<double, std::vector<ReplaySensorType>> _mea_seq;
  std::map<double, std::vector<ReplaySensorType>>::iterator _mea_it;
  int _mea_id{0};
};
}  // namespace apa_slam
