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
    std::vector<patac_hpp::DRPose> ReadDrPose(int&rc );
    
    // 读取停车位数据
    std::vector<patac_hpp::ParkingSlotList> ReadParkingSlots();
  
    // 读取img数据
    std::vector<patac_hpp::ImageList> ReadImgs(int&rc);
    
private:
    sqlite3* _db;
    int _rc;
    char* _errMsg;
    sqlite3_stmt* _stmt_imgs = nullptr;  // 保存图片查询状态
    bool _initialized_imgs = false;      // 标记图片是否已初始化查询
};
} // namespace apa_slam
