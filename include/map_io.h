#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Eigen>
#include "patac_trajetory.pb.h"
#include "patac_slot.pb.h"

namespace apa_slam {

class MapIO {
public:

  static MapIO& GetInstance();

  bool SaveMapData();
  
  bool LoadMapData(const std::string& map_file_path);
  
  void CollectDataFromManagers();
  
  void ClearCollectedData();
  
  void ClearManagerCache();

  bool SetSemMap();

private:
  struct EulerAngles {
    double roll;
    double pitch;
    double yaw;
  };

  std::vector<patac_hpp::ParkingSlotList> margin_data_list_;
  std::vector<patac_hpp::ParkingSlotList> slot_map_data_list_;
  patac_hpp::Trajectory deleted_window_data_list_;

  bool WriteTrajectoryToCSV(const std::string &csv_file_path);
  double QuaternionToYaw(double qx, double qy, double qz, double qw);
  EulerAngles QuaternionToEulerAngles(double qx, double qy, double qz, double qw);

  // 内部序列化和写入方法
  bool WriteMapDataToBinary(const std::string& file_path);
  bool WriteWindowDataToBinary(const std::string& file_path);
  
  // 内部读取和反序列化方法
  bool ReadMapDataFromBinary(const std::string& file_path);
  bool ReadWindowDataFromBinary(const std::string& file_path);
  
  // 序列化辅助方法
  template<typename T>
  bool SerializeAndWrite(std::ofstream& file, const std::vector<T>& data_list);
  template<typename T>
  bool ReadAndDeserialize(std::ifstream& file, std::vector<T>& data_list);

};

} // namespace apa_slam