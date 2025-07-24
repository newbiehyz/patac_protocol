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

std::vector<patac_hpp::DRPose> DataReader::ReadDrPose() {
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

std::vector<patac_hpp::ImageList> DataReader::ReadImgs(){

    std::vector<patac_hpp::ImageList> result;
    std::string query = "SELECT timestamp, data FROM fisheye_front ORDER BY timestamp;";
    
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
        
        patac_hpp::ImageList imgs;
        if (blob_data && blob_size > 0) {
            if (imgs.ParseFromArray(blob_data, blob_size)) {
                result.push_back(imgs);
            } else {
                std::cerr << "Failed to parse ParkingSlotList at timestamp: " << timestamp << "\n";
            }
        }
    }
    
    sqlite3_finalize(stmt);
    return result;

}

} // namespace apa_slam