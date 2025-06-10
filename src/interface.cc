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
  this->Reset();

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

void LocalMappingInterface::ProcDrPose(long long timestamp,
                                       const Eigen::VectorXd &pose) {
  apa_slam::Pose p;
  p.x = pose.x();
  p.y = pose.y();
  p.yaw = pose.z();
  std::ofstream fout_interface;
  fout_interface.open(_output_file_name, std::ios::app);
  double timestamp_d = static_cast<double>(timestamp) * 0.001;
  fout_interface << "pose " << std::setprecision(20)
                 << std::to_string(timestamp_d) << " " << p.x << " " << p.y
                 << " " << p.yaw << std::endl;
  fout_interface.close();
  double v_out, w_out;
  long long ts_out;
  if (EkfEstimator::GetInstance().ProcDrPose(timestamp, p, ts_out, v_out,
                                             w_out)) {
    Eigen::VectorXd mea_data = Eigen::VectorXd::Zero(2);
    mea_data[0] = v_out;
    mea_data[1] = w_out;
    // std::cout << "v_out: " << v_out << std::endl;
    KinematicMea::Ptr odo_mea =
        std::make_shared<OdoMea>(ts_out, mea_data.data());
    std::vector<KinematicMea::Ptr> mea_vector;
    mea_vector.push_back(odo_mea);
    EkfEstimator::GetInstance().InputKinematicMea(ts_out, mea_vector);
  }
}

void LocalMappingInterface::ProcSlotData(
    long long timestamp, const std::vector<Eigen::VectorXd> &slot_data,
    const std::vector<ParkingSlotAttribute> &slot_attribute) {
  std::vector<SemanticMea::Ptr> slot_meas;
  std::ofstream fout_interface;
  fout_interface.open(_output_file_name, std::ios::app);
  double timestamp_d = static_cast<double>(timestamp) * 0.001;

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

    if (pt0.norm() > ApaParameters::GetInstance()
                         .GetEstimatorParamters()
                         .slot_mea_max_range ||
        pt1.norm() > ApaParameters::GetInstance()
                         .GetEstimatorParamters()
                         .slot_mea_max_range) {
      continue;
    }

    data.col(0) = pt0;
    data.col(1) = pt1;

    Eigen::Vector2d dir = pt1 - pt0;
    dir.normalize();
    const auto &inner_tunning = ApaParameters::GetInstance()
                                    .GetEstimatorParamters()
                                    .slot_inward_tunning;
    pt0 += dir * inner_tunning;
    pt1 -= dir * inner_tunning;
    fout_interface << " " << pt0.x() << " " << pt0.y() << " " << pt1.x() << " "
                   << pt1.y();
    SemanticMea::Ptr mea =
        std::make_shared<ParkingSlotMea>(timestamp, data.data());
    auto slot_mea = std::dynamic_pointer_cast<ParkingSlotMea>(mea);
    slot_mea->SetAttribute(slot_attribute.at(i));
    slot_meas.push_back(mea);
  }
  fout_interface << std::endl;
  fout_interface.close();

  EkfEstimator::GetInstance().InputSemanticMea(timestamp, slot_meas);
}

bool LocalMappingInterface::GetLatestVehiclePose(Eigen::VectorXd &pose) {
  if (!EkfEstimator::GetInstance().Initialized()) {
    return false;
  }

  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  long long timestamp;
  if (EkfEstimator::GetInstance().GetLatestVechileState(timestamp, x, P)) {
    pose = x;
    return true;
  }

  return false;
}

bool LocalMappingInterface::GetLatestSlotMap(
    std::map<int, Eigen::MatrixXd> &slot_map,
    std::map<int, ParkingSlotAttribute> &slot_attri
  ) {
  if (!SemanticMap::GetInstance().HasMap(SEMANTIC_TYPE_PARKING_SLOT)) {
    return false;
  }
  const auto &map =
      SemanticMap::GetInstance().GetMap(SEMANTIC_TYPE_PARKING_SLOT);
  for (auto it = map.begin(); it != map.end(); ++it) {
    if (it->second->Initialized()) {
      auto slot = std::dynamic_pointer_cast<ParkingSlotLandmark>(it->second);
      Eigen::MatrixXd slot_data = slot->ConstructFullSlot();
      int id = it->second->GetId();
      slot_map[id] = slot_data;
      slot_attri[id] = slot->GetAttribute();
    }
  }

  return true;
}

void LocalMappingInterface::Reset() { EkfEstimator::GetInstance().Reset(); }

}  // namespace apa_slam
