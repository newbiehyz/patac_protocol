#include "data_reader.h"
#include <sqlite3.h>
#include <iostream>

namespace apa_slam {

DataReader::DataReader(const std::string& db_path) {
    _rc = sqlite3_open(db_path.c_str(), &_db);
    if (_rc) {
        std::cerr << "Can't open database: " << sqlite3_errmsg(_db) << "\n";
    }
}

DataReader::~DataReader() {
    sqlite3_close(_db);
    if (_stmt_imgs) {
        sqlite3_finalize(_stmt_imgs);
    }
    if (_stmt_seq) {
        sqlite3_finalize(_stmt_seq);
    }
    if (_stmt_dr) {
        sqlite3_finalize(_stmt_dr);
    }
    if (_stmt_slots) {
        sqlite3_finalize(_stmt_slots);
    }
    _initialized_seq = false;
    _initialized_dr = false;
    _initialized_slots = false;
    _initialized_imgs = false;
    for (int i = 0; i < 4; ++i) {
        _initialized_query[i] = false;
        if (_stmt_query[i]) {
            sqlite3_finalize(_stmt_query[i]);
        }
    }  
}

std::vector<std::pair<int64_t, DataType>> DataReader::ReadDataSeq(int&rc) {
    std::vector<std::pair<int64_t, DataType>> result;
    // 第一次调用时准备查询
    if (!_initialized_seq) {
        std::string query = "SELECT timestamp, type FROM data_seq ORDER BY timestamp;";
        _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &_stmt_seq, nullptr);
            
        if (_rc != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
            return result;
        }
        _initialized_seq = true;
    }
    
    if ((_rc = sqlite3_step(_stmt_seq)) == SQLITE_ROW) {
        int64_t timestamp = sqlite3_column_int64(_stmt_seq, 0);
        int type = sqlite3_column_int(_stmt_seq, 1);
        result.emplace_back(timestamp, static_cast<DataType>(type));
    }
    rc = _rc;
    // 如果到达结果集末尾，清理资源
    if (_rc != SQLITE_ROW) {
        sqlite3_finalize(_stmt_seq);
        _stmt_seq = nullptr;
        _initialized_seq = false;
    }
    return result;
}

std::vector<patac_hpp::DRPose> DataReader::ReadDrPose(int&rc) {
    std::vector<patac_hpp::DRPose> result;
    // 第一次调用时准备查询
    if (!_initialized_dr) {
        std::string query = "SELECT timestamp, data FROM dr_pose ORDER BY timestamp;";
        _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &_stmt_dr, nullptr);
            
        if (_rc != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
            return result;
        }
        _initialized_dr = true;
    }

    if ((_rc = sqlite3_step(_stmt_dr)) == SQLITE_ROW) {

        int64_t timestamp = sqlite3_column_int64(_stmt_dr, 0);
        const void* blob_data = sqlite3_column_blob(_stmt_dr, 1);
        int blob_size = sqlite3_column_bytes(_stmt_dr, 1);

        patac_hpp::DRPose pose;   
        if (blob_data && blob_size > 0) {
            if (pose.ParseFromArray(blob_data, blob_size)) {
                result.push_back(pose);
            } else {
                std::cerr << "Failed to parse DRPose at timestamp: " << timestamp << "\n";
            }
        }
    }    
    rc = _rc;
    // 如果到达结果集末尾，清理资源
    if (_rc != SQLITE_ROW) {
        sqlite3_finalize(_stmt_dr);
        _stmt_dr = nullptr;
        _initialized_dr = false;
    }
    return result;
}

std::vector<patac_hpp::ParkingSlotList> DataReader::ReadParkingSlots(int&rc) {
    std::vector<patac_hpp::ParkingSlotList> result;
    // 第一次调用时准备查询
    if (!_initialized_slots) {
        std::string query = "SELECT timestamp, data FROM slot_list ORDER BY timestamp;";
        _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &_stmt_slots, nullptr);
            
        if (_rc != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
            return result;
        }
        _initialized_slots = true;
    }
        
    if ((_rc = sqlite3_step(_stmt_slots)) == SQLITE_ROW) {

        int64_t timestamp = sqlite3_column_int64(_stmt_slots, 0);
        const void* blob_data = sqlite3_column_blob(_stmt_slots, 1);
        int blob_size = sqlite3_column_bytes(_stmt_slots, 1);

        patac_hpp::ParkingSlotList slots;
        
        if (blob_data && blob_size > 0) {
            if (slots.ParseFromArray(blob_data, blob_size)) {
                slots.set_timestamp(timestamp);// db col[0](timestamp)  
                result.push_back(slots);
            } else {
                std::cerr << "Failed to parse ParkingSlotList at timestamp: " << timestamp << "\n";
            }
        }
    }   
    rc = _rc;
    // 如果到达结果集末尾，清理资源
    if (_rc != SQLITE_ROW) {
        sqlite3_finalize(_stmt_slots);
        _stmt_slots = nullptr;
        _initialized_slots = false;
    }
    return result;
}

std::vector<patac_hpp::ImageList> DataReader::ReadImgs(int&rc){
    std::vector<patac_hpp::ImageList> result;
    // 第一次调用时准备查询
    if (!_initialized_imgs) {
        std::string query = "SELECT timestamp, data FROM fisheye_images ORDER BY timestamp;";
        _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &_stmt_imgs, nullptr);
            
        if (_rc != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
            return result;
        }
        _initialized_imgs = true;
    }
    
    if ((_rc = sqlite3_step(_stmt_imgs)) == SQLITE_ROW) {
        int64_t timestamp = sqlite3_column_int64(_stmt_imgs, 0);
        const void* blob_data = sqlite3_column_blob(_stmt_imgs, 1);
        int blob_size = sqlite3_column_bytes(_stmt_imgs, 1);

        patac_hpp::ImageList imgs;
        
        if (blob_data && blob_size > 0) {
            if (imgs.ParseFromArray(blob_data, blob_size)) {
                imgs.set_timestamp(timestamp);
                result.push_back(imgs);
            } else {
                std::cerr << "Failed to parse ParkingSlotList at timestamp: " << timestamp << "\n";
            }
        }
    }
    rc = _rc;
    // 如果到达结果集末尾，清理资源
    if (_rc != SQLITE_ROW) {
        sqlite3_finalize(_stmt_imgs);
        _stmt_imgs = nullptr;
        _initialized_imgs = false;
    }
    
    return result;

}


} // namespace apa_slam