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

  // timestamp for filename
  std::time_t now = std::time(nullptr);
  std::tm *local_time = std::localtime(&now);
  std::ostringstream time_str;
  time_str << std::put_time(local_time, "%Y-%m-%d_%H-%M-%S");
  std::string timestamp = time_str.str();

  CollectDataFromManagers();

  if (margin_data_list_.empty() && slot_map_data_list_.empty() &&
      deleted_window_data_list_.empty()) {
    std::cout << "No data to save" << std::endl;
    return true;
  }

  // 1. save margin + slot map
  if (!margin_data_list_.empty() || !slot_map_data_list_.empty()) {
    std::string map_file_path = timestamp + "_map_data.bin";
    if (!WriteMapDataToBinary(map_file_path)) {
      std::cerr << "Failed to save map data" << std::endl;
      return false;
    }
  }

  // 2. save deleted window data
  if (!deleted_window_data_list_.empty()) {
    std::string window_file_path = timestamp + "_deleted_window_data.bin";
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
bool MapIO::LoadMapData(const std::string &map_file_path,
                        const std::string &window_file_path) {

  ClearCollectedData();

  if (!map_file_path.empty()) {
    if (!ReadMapDataFromBinary(map_file_path)) {
      std::cerr << "Failed to load map data" << std::endl;
      return false;
    }
  }

  if (!window_file_path.empty()) {
    if (!ReadWindowDataFromBinary(window_file_path)) {
      std::cerr << "Failed to load window data" << std::endl;
      return false;
    }
  }

  std::cout << "Map data loaded successfully" << std::endl;

  return true;
}

void MapIO::CollectDataFromManagers() {
  margin_data_list_ =
      SlEKFManagement::GetInstance().GetCachedMarginalizationData();
  slot_map_data_list_ = SlEKFManagement::GetInstance().GetCachedSlotMapData();
  deleted_window_data_list_ =
      SlidingWindow::GetInstance().GetCachedDeletedWindowData();
}

void MapIO::ClearCollectedData() {
  margin_data_list_.clear();
  slot_map_data_list_.clear();
  deleted_window_data_list_.clear();
}

void MapIO::ClearManagerCache() {
  SlEKFManagement::GetInstance().ClearMarginalizationDataCache();
  SlidingWindow::GetInstance().ClearDeletedWindowCache();
  SlEKFManagement::GetInstance().ClearSlotMapDataCache();
}

bool MapIO::WriteMapDataToBinary(const std::string &file_path) {
  std::ofstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for writing" << std::endl;
    return false;
  }

  if (!SerializeAndWrite(file, margin_data_list_)) {
    file.close();
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

  bool success = SerializeAndWrite(file, deleted_window_data_list_);

  file.close();
  return success;
}

bool MapIO::ReadMapDataFromBinary(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open " << file_path << " for reading" << std::endl;
    return false;
  }

  if (!ReadAndDeserialize(file, margin_data_list_)) {
    file.close();
    return false;
  }

  if (!ReadAndDeserialize(file, slot_map_data_list_)) {
    file.close();
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

  bool success = ReadAndDeserialize(file, deleted_window_data_list_);

  file.close();
  return success;
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