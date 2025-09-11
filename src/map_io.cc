/*
 * Filename: map_io.cc
 * Author: yukan
 * Description: MapIO class implementation
 */

#include "map_io.h"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "sl_ekf_management.h"
#include "sliding_window.h"

namespace apa_slam {

MapIO &MapIO::GetInstance() {
  static MapIO instance;
  return instance;
}

bool MapIO::SaveMapData() {
  CollectDataFromManagers();

  if (slot_map_data_list_.empty() &&
      deleted_window_data_list_.trajectory_point_list_size() == 0) {
    std::cout << "No data to save" << std::endl;
    return true;
  }

  const auto &map_io_params = ApaParameters::GetInstance().GetMapIOParameters();

  // 1. save slot map
  if (!slot_map_data_list_.empty()) {
    std::string map_file_path = map_io_params.map_save_path + "sematic_map.bin";
    if (!WriteMapDataToBinary(map_file_path)) {
      std::cerr << "Failed to save map data" << std::endl;
      return false;
    }
  }

  // 2. save deleted window data
  if (deleted_window_data_list_.trajectory_point_list_size() > 0) {
    std::string window_file_path;
    if (vis_meas.startMapping){
       window_file_path = map_io_params.map_save_path + "trajectory_map.bin";
    } else{
       window_file_path = map_io_params.map_save_path + "trajectory_loc.bin";
    } 

    if (!WriteWindowDataToBinary(window_file_path)) {
      std::cerr << "Failed to save window data" << std::endl;
      return false;
    }
  }

  ClearManagerCache();
  std::cout << "Map data saved successfully" << std::endl;

  return true;
}

bool MapIO::LoadMapData(const std::string &input_map_file_path) {
  ClearCollectedData();

  if (!input_map_file_path.empty()) {
    if (!ReadMapDataFromBinary(input_map_file_path)) {
      std::cerr << "Failed to load map data from: " << input_map_file_path
                << std::endl;
      return false;
    }
  }

  std::cout << "Map data loaded successfully" << std::endl;

  return true;
}

void MapIO::CollectDataFromManagers() {
  slot_map_data_list_ = SlEKFManagement::GetInstance().GetCachedSlotMapData();
  deleted_window_data_list_ = SlidingWindow::GetInstance().GetCachedDeletedWindowData();
}

void MapIO::ClearCollectedData() {
  slot_map_data_list_.clear();
  deleted_window_data_list_.Clear();
}

void MapIO::ClearManagerCache() {
  SlidingWindow::GetInstance().ClearDeletedWindowCache();
  SlEKFManagement::GetInstance().ClearSlotMapDataCache();
}


bool MapIO::WriteTrajectoryToCSV(const std::string &csv_file_path) {
  std::ofstream csv_file(csv_file_path);

  if (!csv_file.is_open()) {
    std::cerr << "Failed to open " << csv_file_path << " for writing"
              << std::endl;
    return false;
  }

  csv_file << "timestamp,x,y,z,roll,yaw,pitch" << std::endl;

  for (int i = 0; i < deleted_window_data_list_.trajectory_point_list_size(); ++i) {
    const auto& point = deleted_window_data_list_.trajectory_point_list(i);
    
    // 将四元数转换为欧拉角
    auto euler_angles = QuaternionToEulerAngles(point.qx(), point.qy(), point.qz(), point.qw());
    
    // 转换为度数
    double roll_degrees = euler_angles.roll * 180.0 / M_PI;
    double pitch_degrees = euler_angles.pitch * 180.0 / M_PI;
    double yaw_degrees = euler_angles.yaw * 180.0 / M_PI;

    csv_file << point.timestamp() << "," 
             << std::fixed << std::setprecision(6) 
             << point.x() << "," 
             << point.y() << "," 
             << point.z() << "," 
             << roll_degrees << "," 
             << yaw_degrees << "," 
             << pitch_degrees << std::endl;
  }

  csv_file.close();
  return true;
}

double MapIO::QuaternionToYaw(double qx, double qy, double qz, double qw) {
  Eigen::Quaterniond quaternion(qw, qx, qy,qz);
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
  double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

  return yaw;
}

MapIO::EulerAngles MapIO::QuaternionToEulerAngles(double qx, double qy, double qz, double qw) {
    EulerAngles angles;
    
    double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
    
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // 使用90度如果超出范围
    else
        angles.pitch = std::asin(sinp);
    
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return angles;
}

bool MapIO::WriteMapDataToBinary(const std::string &file_path) {
  std::ofstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for writing" << std::endl;
    return false;
  }

  if (!SerializeAndWrite(file, slot_map_data_list_)) {
    file.close();
    return false;
  }

  file.close();
  return true;
}

bool MapIO::WriteWindowDataToBinary(const std::string &file_path) {
  std::ofstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for writing" << std::endl;
    return false;
  }

  std::string serialized_data;
  if (!deleted_window_data_list_.SerializeToString(&serialized_data)) {
    std::cerr << "Failed to serialize trajectory data" << std::endl;
    file.close();
    return false;
  }

  // 写入数据长度
  uint32_t data_length = serialized_data.size();
  file.write(reinterpret_cast<const char *>(&data_length), sizeof(data_length));

  // 写入序列化数据
  file.write(serialized_data.data(), serialized_data.size());

  file.close();

  // 保存CSV文件
  std::string csv_file_path = file_path;
  size_t pos = csv_file_path.find_last_of('.');
  if (pos != std::string::npos) {
    csv_file_path = csv_file_path.substr(0, pos) + ".csv";
  } else {
    csv_file_path += ".csv";
  }

  if (!WriteTrajectoryToCSV(csv_file_path)) {
    std::cerr << "Failed to write CSV file: " << csv_file_path << std::endl;
  }

  return true;
}

bool MapIO::SetSemMap() {
  for (int i = 0; i < slot_map_data_list_.size(); i++) {
    patac_hpp::ParkingSlotList slot_list = slot_map_data_list_[i];
    for (int i = 0; i < slot_list.parking_slot_list_size(); ++i) {
      const auto &parking_slot = slot_list.parking_slot_list(i);
      Eigen::MatrixXd slot_data =
          Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT);
      slot_data(0, 0) = parking_slot.points(0).x();
      slot_data(1, 0) = parking_slot.points(0).y();
      slot_data(0, 1) = parking_slot.points(1).x();
      slot_data(1, 1) = parking_slot.points(1).y();

      SemanticLandmark::Ptr landmark = std::make_shared<ParkingSlotLandmark>(
          parking_slot.id(), slot_data.data());
      auto slot = std::dynamic_pointer_cast<ParkingSlotLandmark>(landmark);
      slot->SetInitializeFlag(true);
      auto attr = slot->GetAttribute();

      switch (parking_slot.type()) {
        case patac_hpp::SlotTypeVertical:
          attr.slot_type = Vertical;
          break;
        case patac_hpp::SlotTypeParallel:
          attr.slot_type = Horizontal;
          break;
        default:
          attr.slot_type = Oblique;
          break;
      }

      attr.parkable =
          (parking_slot.occupancy() == patac_hpp::OccupancyStatusNotOccupied);
      slot->SetAttribute(attr);
      SemanticMap::GetInstance().AddLandmark(landmark->GetSemanticType(),
                                             landmark);
    }
  }
  std::cout << "[Load Map] Map Element Num: "
            << SemanticMap::GetInstance().GetMapLandmarkNum(
                   SEMANTIC_TYPE_PARKING_SLOT)
            << std::endl;

  std::cout << "[Load Map] Map Initialized Element Num: "
            << SemanticMap::GetInstance().GetMapInitializedLandmarkNum(
                   SEMANTIC_TYPE_PARKING_SLOT)
            << std::endl;

  std::cout
      << "Loaded "
      << SemanticMap::GetInstance().GetMap(SEMANTIC_TYPE_PARKING_SLOT).size()
      << " parking slots "
      << std::endl;
  return true;
}

bool MapIO::ReadMapDataFromBinary(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for reading" << std::endl;
    return false;
  }

  if (!ReadAndDeserialize(file, slot_map_data_list_)) {
    file.close();
    return false;
  }

  if (!SetSemMap()) {
    std::cerr << "Failed to SetSemMap" << std::endl;
    return false;
  }

  file.close();
  return true;
}

bool MapIO::ReadWindowDataFromBinary(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for reading" << std::endl;
    return false;
  }

  // 读取数据长度
  uint32_t data_length;
  file.read(reinterpret_cast<char *>(&data_length), sizeof(data_length));

  if (file.eof()) {
    file.close();
    return false;
  }

  // 读取序列化数据
  std::vector<char> buffer(data_length);
  file.read(buffer.data(), data_length);

  // 反序列化
  if (!deleted_window_data_list_.ParseFromArray(buffer.data(), data_length)) {
    std::cerr << "Failed to deserialize trajectory data" << std::endl;
    file.close();
    return false;
  }

  file.close();
  return true;
}

template <typename T>
bool MapIO::SerializeAndWrite(std::ofstream &file,
                              const std::vector<T> &data_list) {
  for (const auto &data : data_list) {
    std::string serialized_data;
    if (!data.SerializeToString(&serialized_data)) {
      std::cerr << "Failed to serialize data" << std::endl;
      return false;
    }

    // 写入数据长度
    uint32_t data_length = serialized_data.size();
    file.write(reinterpret_cast<const char *>(&data_length),
               sizeof(data_length));

    // 写入序列化数据
    file.write(serialized_data.data(), serialized_data.size());
  }

  return true;
}

template <typename T>
bool MapIO::ReadAndDeserialize(std::ifstream &file, std::vector<T> &data_list) {
  while (file.good()) {
    // 读取数据长度
    uint32_t data_length;
    file.read(reinterpret_cast<char *>(&data_length), sizeof(data_length));

    if (file.eof()) break;

    // 读取序列化数据
    std::vector<char> buffer(data_length);
    file.read(buffer.data(), data_length);

    // 反序列化
    T data;
    if (!data.ParseFromArray(buffer.data(), data_length)) {
      std::cerr << "Failed to deserialize data" << std::endl;
      return false;
    }

    data_list.push_back(data);
  }

  return true;
}

}  // namespace apa_slam