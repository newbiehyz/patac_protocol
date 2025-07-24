/*
 * Copyright (c) 2025 h
 */

#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sqlite3.h>
#include <vector>

#include "patac_dr.pb.h"
#include "patac_image.pb.h"
#include "patac_slot.pb.h"

namespace apa_slam {

enum DataType : std::uint8_t { Pose = 0, DR = 1, SLOT = 2, FISH_FRONT = 3 };

class DataReader {
public:
    DataReader(const std::string& db_path);
    ~DataReader();
    
    // 读取数据序列
    std::vector<std::pair<int64_t, DataType>> ReadDataSeq();
    
    // 读取DR姿态数据
    std::vector<patac_hpp::DRPose> ReadDrPose();
    
    // 读取停车位数据
    std::vector<patac_hpp::ParkingSlotList> ReadParkingSlots();
  
    // 读取img数据
    std::vector<patac_hpp::ImageList> ReadImgs();
    
private:
    sqlite3* _db;
    int _rc;
    char* _errMsg;
};
} // namespace apa_slam
