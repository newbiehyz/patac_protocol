#include "data_writer.h"
namespace apa_slam {
DataWriter::DataWriter() {}

DataWriter &DataWriter::GetInstance() {
  static DataWriter instance;
  return instance;
}

void DataWriter::Init() {
  std::time_t now = std::time(nullptr);
  std::tm *localTime = std::localtime(&now);

  std::ostringstream oss;
  oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
  //   _output_file_name = "/userdata/apatest/" + oss.str() + ".db";
  // _output_file_name = "/home/yukan/Desktop/" + oss.str() + ".db";
  _output_file_name = "/userdata/apatest/" + oss.str() + ".db";
  _rc = sqlite3_open(_output_file_name.c_str(), &_db);
  if (_rc) {
    std::cerr << "Can't open database: " << sqlite3_errmsg(_db) << "\n";
    return;
  }

  create_tables();

  // create table of message recv
}

void DataWriter::create_tables() {
  std::string create_data_seq_tbl = R"(
        CREATE TABLE IF NOT EXISTS data_seq (
            timestamp INTEGER NOT NULL,
            type INTEGER NOT NULL
        );
    )";

  _rc = sqlite3_exec(_db, create_data_seq_tbl.c_str(), nullptr, nullptr,
                     &_errMsg);

  if (_rc != SQLITE_OK) {
    std::cerr << "create_data_seq_tbl failed SQL error: " << _errMsg
              << std::endl;
    sqlite3_free(_errMsg);
    sqlite3_close(_db);
    return;
  }
  std::cout << "create_data_seq_tbl Succ\n";

  std::string create_image_tbl = R"(CREATE TABLE IF NOT EXISTS fisheye_images (
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);)";

  _rc = sqlite3_exec(_db, create_image_tbl.c_str(), nullptr, nullptr, &_errMsg);

  if (_rc != SQLITE_OK) {
    std::cerr << "create_image_tbl failed SQL error: " << _errMsg << std::endl;
    sqlite3_free(_errMsg);
    sqlite3_close(_db);
    return;
  }

  std::cout << "create_image_tbl Succ\n";

  std::string create_dr_tbl = R"(CREATE TABLE IF NOT EXISTS dr_pose (
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);)";

  _rc = sqlite3_exec(_db, create_dr_tbl.c_str(), nullptr, nullptr, &_errMsg);

  if (_rc != SQLITE_OK) {
    std::cerr << "create_dr_tbl failed SQL error: " << _errMsg << std::endl;
    sqlite3_free(_errMsg);
    sqlite3_close(_db);
    return;
  }

  std::cout << "create_dr_tbl Succ\n";

  std::string create_slot_tbl = R"(CREATE TABLE IF NOT EXISTS slot_list (
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);)";

  _rc = sqlite3_exec(_db, create_slot_tbl.c_str(), nullptr, nullptr, &_errMsg);

  if (_rc != SQLITE_OK) {
    std::cerr << "create_slot_tbl failed SQL error: " << _errMsg << std::endl;
    sqlite3_free(_errMsg);
    sqlite3_close(_db);
    return;
  }

  std::cout << "create_slot_tbl Succ\n";
}

void DataWriter::WriteDataSeq(const long long timestamp, const DataType &type) {
  std::string insert_data_seq =
      "INSERT INTO data_seq (timestamp, type) VALUES (?, ?);";
  sqlite3_stmt *stmt_seq = nullptr;
  sqlite3_prepare_v2(_db, insert_data_seq.c_str(), -1, &stmt_seq, nullptr);

  sqlite3_bind_int64(stmt_seq, 1, timestamp);
  sqlite3_bind_int(stmt_seq, 2, type);

  if (sqlite3_step(stmt_seq) != SQLITE_DONE) {
    std::cerr << "Insert into data_seq failed: " << sqlite3_errmsg(_db)
              << std::endl;
  }

  sqlite3_reset(stmt_seq);
  sqlite3_finalize(stmt_seq);
}

void DataWriter::Reset() {
  sqlite3_close(_db);
  Init();
}

void DataWriter::WriteDrPoseMsg(const long long timestamp,
                                const Eigen::Vector3d &pose) {
  std::string insert_data_dr =
      "INSERT INTO dr_pose (timestamp, data) VALUES (?, ?);";
  patac_hpp::DRPose dr_pose;
  dr_pose.set_timestamp(timestamp);
  dr_pose.set_x(pose.x());
  dr_pose.set_y(pose.y());
  dr_pose.set_yaw(pose.z());
  std::string serialized;
  dr_pose.SerializeToString(&serialized);

  sqlite3_stmt *stmt_seq = nullptr;
  sqlite3_prepare_v2(_db, insert_data_dr.c_str(), -1, &stmt_seq, nullptr);

  sqlite3_bind_int64(stmt_seq, 1, timestamp);
  sqlite3_bind_blob(stmt_seq, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(stmt_seq);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into dr_pose failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  // if (sqlite3_step(stmt_seq) != SQLITE_DONE) {
  //   std::cerr << "Insert into dr_pose failed: " << sqlite3_errmsg(_db)
  //             << std::endl;
  // }

  sqlite3_reset(stmt_seq);
  sqlite3_finalize(stmt_seq);
}

void DataWriter::WriteImageList(const long long timestamp,
                                const std::vector<cv::Mat> &imgs) {
  std::string insert_data_img =
      "INSERT INTO fisheye_images (timestamp, data) VALUES (?, ?);";

  patac_hpp::ImageList img_list;
  for (size_t i = 0; i < imgs.size(); ++i) {
    auto image = img_list.add_image_list();
    image->set_width(imgs.at(i).cols);
    image->set_height(imgs.at(i).rows);
    std::vector<uchar> buffer;
    cv::imencode(".png", imgs.at(i), buffer);
    // size_t data_size = imgs.at(i).total() * imgs.at(i).elemSize();
    image->set_data(buffer.data(), buffer.size());
  }

  std::string serialized;
  img_list.SerializeToString(&serialized);

  sqlite3_stmt *stmt_seq = nullptr;
  sqlite3_prepare_v2(_db, insert_data_img.c_str(), -1, &stmt_seq, nullptr);

  sqlite3_bind_int64(stmt_seq, 1, timestamp);
  sqlite3_bind_blob(stmt_seq, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(stmt_seq);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into fisheye_images failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  // if (sqlite3_step(stmt_seq) != SQLITE_DONE) {
  //   std::cerr << "Insert into dr_pose failed: " << sqlite3_errmsg(_db)
  //             << std::endl;
  // }

  sqlite3_reset(stmt_seq);
  sqlite3_finalize(stmt_seq);
}

void DataWriter::WriteSlotMsg(const long long timestamp,
                              const std::vector<Eigen::MatrixXd> &slot_uv) {
  std::string insert_data_ps =
      "INSERT INTO slot_list (timestamp, data) VALUES (?, ?);";
  patac_hpp::ParkingSlotList ps_list;
  for (size_t i = 0; i < slot_uv.size(); ++i) {
    auto *ps = ps_list.add_parking_slot_list();

    for (size_t j = 0; j < 4; ++j) {
      auto *pt = ps->add_points();
      pt->set_x(slot_uv.at(i).col(j).x());
      pt->set_y(slot_uv.at(i).col(j).y());
    }
  }

  std::string serialized;
  ps_list.SerializeToString(&serialized);

  sqlite3_stmt *stmt_seq = nullptr;
  sqlite3_prepare_v2(_db, insert_data_ps.c_str(), -1, &stmt_seq, nullptr);

  sqlite3_bind_int64(stmt_seq, 1, timestamp);
  sqlite3_bind_blob(stmt_seq, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(stmt_seq);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into slot_list failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  // if (sqlite3_step(stmt_seq) != SQLITE_DONE) {
  //   std::cerr << "Insert into dr_pose failed: " << sqlite3_errmsg(_db)
  //             << std::endl;
  // }

  sqlite3_reset(stmt_seq);
  sqlite3_finalize(stmt_seq);
}

void DataWriter::WriteSlotMsg(const long long timestamp,
                              const class patac_hpp::ParkingSlotList &slot_uv) {
  std::string insert_data_slot =
      "INSERT INTO slot_list (timestamp, data) VALUES (?, ?);";

  patac_hpp::ParkingSlotList ps_list;
  ps_list = slot_uv;
  std::string serialized;
  ps_list.SerializeToString(&serialized);
  sqlite3_stmt *stmt_seq = nullptr;
  sqlite3_prepare_v2(_db, insert_data_slot.c_str(), -1, &stmt_seq, nullptr);

  sqlite3_bind_int64(stmt_seq, 1, timestamp);
  sqlite3_bind_blob(stmt_seq, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(stmt_seq);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into slot_list failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  // if (sqlite3_step(stmt_seq) != SQLITE_DONE) {
  //   std::cerr << "Insert into dr_pose failed: " << sqlite3_errmsg(_db)
  //             << std::endl;
  // }

  sqlite3_reset(stmt_seq);
  sqlite3_finalize(stmt_seq);
}

}  // namespace apa_slam
