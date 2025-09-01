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
#include "patac_ptcloud.pb.h"
#include "data_type.h"
namespace apa_slam {

class DataReader {
public:
    DataReader(const std::string& db_path);
    ~DataReader();
    
    // 顺序读取数据序列
    std::vector<std::pair<int64_t, DataType>> ReadDataSeq(int&rc);
    
    // 顺序读取DR姿态数据
    std::vector<patac_hpp::DRPose> ReadDrPose(int&rc );
    
    // 顺序读取停车位数据
    std::vector<patac_hpp::ParkingSlotList> ReadParkingSlots(int&rc);
  
    // 顺序读取img数据
    std::vector<patac_hpp::ImageList> ReadImgs(int&rc);

    // 根据时间戳查询特定类型的完整数据
    template <typename T>
    bool QueryDataByTimestamp(const std::string& table_name, 
                                long long timestamp, 
                                T& result,uint8_t datatype) {
            

        // 第一次调用时准备查询
        if (!_initialized_query[datatype]) {
            std::string query = "SELECT data FROM " + table_name + 
                            " WHERE timestamp = ?;";
            _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &_stmt_query[datatype], nullptr);
                
            if (_rc != SQLITE_OK) {
                std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
                return false;
            }
            _initialized_query[datatype] = true;
        }       
        sqlite3_reset(_stmt_query[datatype]);  
        sqlite3_bind_int64(_stmt_query[datatype], 1, timestamp);
            
        if (sqlite3_step(_stmt_query[datatype]) == SQLITE_ROW) {
            const void* blob = sqlite3_column_blob(_stmt_query[datatype], 0);
            int blob_size = sqlite3_column_bytes(_stmt_query[datatype], 0);
                
            std::string serialized(static_cast<const char*>(blob), blob_size);
            return result.ParseFromString(serialized);
        }
        else {
        sqlite3_finalize(_stmt_query[datatype]);
        _stmt_query[datatype] = nullptr;
        _initialized_query[datatype] = false;
        }     
        return false;

        }   
private:
    sqlite3* _db;
    int _rc;
    char* _errMsg;
    sqlite3_stmt* _stmt_imgs = nullptr;  // 保存图片查询状态
    bool _initialized_imgs = false;      // 标记图片是否已初始化查询
    sqlite3_stmt* _stmt_seq = nullptr;  // 保存seq查询状态
    bool _initialized_seq = false;      // 标记seq是否已初始化查询
    sqlite3_stmt* _stmt_dr = nullptr;  // 保存dr片查询状态
    bool _initialized_dr = false;      // 标记dr是否已初始化查询
    sqlite3_stmt* _stmt_slots = nullptr;  // 保存车位查询状态
    bool _initialized_slots = false;      // 标记车位是否已初始化查询
    sqlite3_stmt* _stmt_query[5] = {nullptr};
    bool _initialized_query[5] = {false};
};
} // namespace apa_slam
