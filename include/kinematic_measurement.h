/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/kinematic_measurement.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 2:29:38 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once
#include <Eigen/Eigen>
#include <memory>

#include "local_mapping_define.h"
namespace apa_slam {
class KinematicMea {
 public:
  typedef std::shared_ptr<KinematicMea> Ptr;
  KinematicMea(const SensorType &type, const double timestamp);
  SensorType GetKineticMeaType();
  virtual ~KinematicMea() = default;
  virtual Eigen::VectorXd GetMeaData() = 0;
  virtual void AddNoise() = 0;

 private:
  SensorType _type;
  double _timestamp;
};
}  // namespace apa_slam
