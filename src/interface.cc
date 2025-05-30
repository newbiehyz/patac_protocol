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
  _cfg = cfg_json;
  ApaParameters::GetInstance().LoadParameters(cfg_json);
  EkfEstimator::GetInstance().Init();

  std::time_t now = std::time(nullptr);
  std::tm *localTime = std::localtime(&now);

  std::ostringstream oss;
  oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
  _output_file_name =
      "/userdata/apatest/" + oss.str() + "_apa_local_mapping.txt";
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
    std::cout << "v_out: " << v_out << std::endl;
    KinematicMea::Ptr odo_mea =
        std::make_shared<OdoMea>(ts_out, mea_data.data());
    std::vector<KinematicMea::Ptr> mea_vector;
    mea_vector.push_back(odo_mea);
    EkfEstimator::GetInstance().InputKinematicMea(ts_out, mea_vector);
  }
}

void LocalMappingInterface::ProcSlotData(
    double timestamp_d, const std::vector<Eigen::VectorXd> &slot_data) {
  std::vector<SemanticMea::Ptr> slot_meas;
  std::ofstream fout_interface;
  fout_interface.open(_output_file_name, std::ios::app);
  fout_interface << "slot " << std::setprecision(20)
                 << std::to_string(timestamp_d) << " " << slot_data.size();
  for (size_t i = 0; i < slot_data.size(); ++i) {
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(2, 2);
    Eigen::Vector2d uv0 = slot_data.at(i).head(2);
    Eigen::Vector2d uv1 = slot_data.at(i).segment(2, 2);
    Eigen::Vector2d pt0 =
        FillbackDataLoader::GetInstance().ConvertUvToVehicle(uv0);
    Eigen::Vector2d pt1 =
        FillbackDataLoader::GetInstance().ConvertUvToVehicle(uv1);
    data.col(0) = pt0;
    data.col(1) = pt1;
    fout_interface << " " << pt0.x() << " " << pt0.y() << " " << pt1.x() << " "
                   << pt1.y();
    SemanticMea::Ptr mea =
        std::make_shared<ParkingSlotMea>(timestamp_d, data.data());
    slot_meas.push_back(mea);
  }
  fout_interface << std::endl;
  fout_interface.close();

  EkfEstimator::GetInstance().InputSemanticMea(timestamp_d, slot_meas);
}

bool LocalMappingInterface::GetLatestVehiclePose(Eigen::VectorXd &pose) {
  if (!EkfEstimator::GetInstance().Initialized()) {
    return false;
  }

  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  double timestamp;
  if (EkfEstimator::GetInstance().GetLatestVechileState(timestamp, x, P)) {
    pose = x;
    return true;
  }

  return false;
}

void LocalMappingInterface::Reset() {
  EkfEstimator::GetInstance().Reset();
  this->Init(_cfg);
}

}  // namespace apa_slam
