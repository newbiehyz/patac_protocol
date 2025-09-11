#pragma once
#include <memory>
#include <thread>
#include "action_queue.h"
#include "apa_parameters.h"
#include "ekf_estimator.h"
#include "fillback_data_loader.h"
#include "kinematic_measurement.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "semantic_measurement.h"
#include "data_writer.h"


namespace apa_slam {

class LocOutput{
public:
  LocOutput();
  static LocOutput &GetInstance();
  double angle_diff(double angle1, double angle2);
  bool GetLatestVechilePos(Eigen::VectorXd& pose);
  void CleanOlderData(double cur_ts);
  bool ProcDrPose(long long ts, const Eigen::VectorXd& pose); //毫秒
  bool Propagate(Eigen::VectorXd &pos, long long timestamp);
  void Reset();

private:
  std::map<long long, DrInfo> _dr_buf; //毫秒
  std::vector<Pose> _dr_pose;
  std::vector<long long> _dr_timestamp; //毫秒

};

}  // namespace apa_slam
