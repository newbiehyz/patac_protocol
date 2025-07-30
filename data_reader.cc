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
}

std::vector<std::pair<int64_t, DataType>> DataReader::ReadDataSeq() {
    std::vector<std::pair<int64_t, DataType>> result;
    std::string query = "SELECT timestamp, type FROM data_seq ORDER BY timestamp;";
    
    sqlite3_stmt* stmt;
    _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &stmt, nullptr);
    
    if (_rc != SQLITE_OK) {
        std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
        return result;
    }
    
    while ((_rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        int64_t timestamp = sqlite3_column_int64(stmt, 0);
        int type = sqlite3_column_int(stmt, 1);
        result.emplace_back(timestamp, static_cast<DataType>(type));
    }
    
    sqlite3_finalize(stmt);
    return result;
}

std::vector<patac_hpp::DRPose> DataReader::ReadDrPose(int&rc) {
    std::vector<patac_hpp::DRPose> result;
    std::string query = "SELECT timestamp, data FROM dr_pose ORDER BY timestamp;";
    
    sqlite3_stmt* stmt;
    _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &stmt, nullptr);
    
    if (_rc != SQLITE_OK) {
        std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
        return result;
    }
    // std::cout << "read _rc: "<<sqlite3_step(stmt)<<" SQLITE_ROW: "<<SQLITE_ROW<<"\n";
    while ((_rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        // if ((_rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        rc = _rc;
        int64_t timestamp = sqlite3_column_int64(stmt, 0);
        const void* blob_data = sqlite3_column_blob(stmt, 1);
        int blob_size = sqlite3_column_bytes(stmt, 1);

        // std::cout << "read loc size: "<<blob_size<<"\n";

        patac_hpp::DRPose pose;
        if (blob_data && blob_size > 0) {
            if (pose.ParseFromArray(blob_data, blob_size)) {
                result.push_back(pose);
            } else {
                std::cerr << "Failed to parse DRPose at timestamp: " << timestamp << "\n";
            }
        }
    }
    
    sqlite3_finalize(stmt);
    return result;
}

std::vector<patac_hpp::ParkingSlotList> DataReader::ReadParkingSlots() {
    std::vector<patac_hpp::ParkingSlotList> result;
    std::string query = "SELECT timestamp, data FROM slot_list ORDER BY timestamp;";
    
    sqlite3_stmt* stmt;
    _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &stmt, nullptr);
    
    if (_rc != SQLITE_OK) {
        std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
        return result;
    }
    
    while ((_rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        int64_t timestamp = sqlite3_column_int64(stmt, 0);
        const void* blob_data = sqlite3_column_blob(stmt, 1);
        int blob_size = sqlite3_column_bytes(stmt, 1);
        
        patac_hpp::ParkingSlotList slots;
        if (blob_data && blob_size > 0) {
            if (slots.ParseFromArray(blob_data, blob_size)) {
                result.push_back(slots);
            } else {
                std::cerr << "Failed to parse ParkingSlotList at timestamp: " << timestamp << "\n";
            }
        }
    }
    
    sqlite3_finalize(stmt);
    return result;
}

std::vector<patac_hpp::ImageList> DataReader::ReadImgs(int&rc){

//     std::vector<patac_hpp::ImageList> result;
//     std::string query = "SELECT timestamp, data FROM fisheye_images ORDER BY timestamp;";
    
//     sqlite3_stmt* stmt;
//     _rc = sqlite3_prepare_v2(_db, query.c_str(), -1, &stmt, nullptr);
    
//     if (_rc != SQLITE_OK) {
//         std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(_db) << "\n";
//         return result;
//     }
    
//     // while ((_rc = sqlite3_step(stmt)) == SQLITE_ROW && result.size() <= 20) {
//     if ((_rc = sqlite3_step(stmt)) == SQLITE_ROW) {
//         rc = _rc;
//         int64_t timestamp = sqlite3_column_int64(stmt, 0);
//         const void* blob_data = sqlite3_column_blob(stmt, 1);
//         int blob_size = sqlite3_column_bytes(stmt, 1);
//         std::cout <<"img timestamp: "<<timestamp<< std::endl;

//         patac_hpp::ImageList imgs;
        
//         if (blob_data && blob_size > 0) {
//             if (imgs.ParseFromArray(blob_data, blob_size)) {
//                 imgs.set_timestamp(timestamp);
//                 result.push_back(imgs);
//             } else {
//                 std::cerr << "Failed to parse ParkingSlotList at timestamp: " << timestamp << "\n";
//             }
//         }
//     }
    
//     sqlite3_finalize(stmt);
//     return result;

// }
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
    
    // while ((_rc = sqlite3_step(stmt)) == SQLITE_ROW && result.size() <= 20) {
    if ((_rc = sqlite3_step(_stmt_imgs)) == SQLITE_ROW) {

        int64_t timestamp = sqlite3_column_int64(_stmt_imgs, 0);
        const void* blob_data = sqlite3_column_blob(_stmt_imgs, 1);
        int blob_size = sqlite3_column_bytes(_stmt_imgs, 1);
        std::cout <<"img timestamp: "<<timestamp<< std::endl;

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