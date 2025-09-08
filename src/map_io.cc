/*
 * Filename: map_io.cc
 * Author: yukan
 * Description: MapIO class implementation
 */

#include "map_io.h"
#include "sl_ekf_management.h"
#include "sliding_window.h"
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace apa_slam {

MapIO &MapIO::GetInstance() {
  static MapIO instance;
  return instance;
}

bool MapIO::SaveMapData() {
  CollectDataFromManagers();

  if (slot_map_data_list_.empty() && deleted_window_data_list_.trajectory_point_list_size() == 0) {
    std::cout << "No data to save" << std::endl;
    return true;
  }

  const auto& map_io_params = ApaParameters::GetInstance().GetMapIOParameters();

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
    std::string window_file_path = map_io_params.map_save_path + "trajectory.bin";
    if (!WriteWindowDataToBinary(window_file_path)) {
      std::cerr << "Failed to save window data" << std::endl;
      return false;
    }
  }

  ClearManagerCache();
  std::cout << "Map data saved successfully" << std::endl;

  return true;
}

// TODO
bool MapIO::LoadMapData(const std::string &input_map_file_path,
                        const std::string &input_window_file_path) {

  ClearCollectedData();

  const auto& map_io_params = ApaParameters::GetInstance().GetMapIOParameters();
  std::string map_file_path = map_io_params.map_load_path + "sematic_map.bin";
  std::string window_file_path = map_io_params.map_load_path + "trajectory.bin";

  if (!map_file_path.empty()) {
    if (!ReadMapDataFromBinary(map_file_path)) {
      std::cerr << "Failed to load map data from: " << map_file_path << std::endl;
      return false;
    }
  }

  if (!window_file_path.empty()) {
    if (!ReadWindowDataFromBinary(window_file_path)) {
      std::cerr << "Failed to load window data from: " << window_file_path << std::endl;
      return false;
    }
  }

  std::cout << "Map data loaded successfully" << std::endl;

  return true;
}

void MapIO::CollectDataFromManagers() {
  slot_map_data_list_ = SlEKFManagement::GetInstance().GetCachedSlotMapData();
  deleted_window_data_list_ = SlEKFManagement::GetInstance().GetCompleteTrajectory();
}

void MapIO::ClearCollectedData() {
  slot_map_data_list_.clear();
  deleted_window_data_list_.Clear();
}

void MapIO::ClearManagerCache() {
  SlidingWindow::GetInstance().ClearDeletedWindowCache();
  SlEKFManagement::GetInstance().ClearSlotMapDataCache();
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
  return true;
}

bool MapIO::SetSemMap(){
  for (int i = 0; i < slot_map_data_list_.size();i++){
    patac_hpp::ParkingSlotList slot_list = slot_map_data_list_[i];
    for (int i = 0; i < slot_list.parking_slot_list_size(); ++i) {
      const auto& parking_slot = slot_list.parking_slot_list(i);
      Eigen::MatrixXd slot_data =
          Eigen::MatrixXd::Zero(DATA_ROWS_PARKING_SLOT, DATA_COLS_PARKING_SLOT);
      slot_data(0, 0) = parking_slot.points(0).x();
      slot_data(1, 0) = parking_slot.points(0).y();
      slot_data(0, 1) = parking_slot.points(1).x();
      slot_data(1, 1) = parking_slot.points(1).y();
      
      SemanticLandmark::Ptr landmark = std::make_shared<ParkingSlotLandmark>(parking_slot.id(), slot_data.data());
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

      attr.parkable = (parking_slot.occupancy() == patac_hpp::OccupancyStatusNotOccupied);
      slot->SetAttribute(attr);
      SemanticMap::GetInstance().AddLandmark(landmark->GetSemanticType(), landmark);
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

  std::cout << "Loaded " << SemanticMap::GetInstance().GetMap(SEMANTIC_TYPE_PARKING_SLOT).size() << " parking slots " 
            << " from " << vis_meas.slot_map_data_filename << std::endl;
  return true;
}

bool MapIO::ReadMapDataFromBinary(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for reading" << std::endl;
    return false;
  }


  
  if (!ReadAndDeserialize(file, slot_map_data_list_))
  {
    file.close();
    return false;
  }
  
  if(!SetSemMap()){
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

    if (file.eof())
      break;

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

} // namespace apa_slam