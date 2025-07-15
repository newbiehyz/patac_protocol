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
  _output_file_name = "/home/yukan/Desktop/" + oss.str() + ".db";

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

  std::string create_image_tbl = R"(CREATE TABLE IF NOT EXISTS fisheye_front (
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
}

void DataWriter::WriteDataSeq(const long long timestamp, const DataType &type) {
  std::string insert_data_seq =
      "INSERT INTO data_seq (timestamp, type) VALUES (?, ?);";
  sqlite3_stmt *stmt_seq = nullptr;
  sqlite3_prepare_v2(_db, insert_data_seq.c_str(), -1, &stmt_seq, nullptr);

  sqlite3_bind_int(stmt_seq, 1, timestamp);
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

  sqlite3_bind_int(stmt_seq, 1, timestamp);
  sqlite3_bind_blob(stmt_seq, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);
}

void DataWriter::WriteFrontImageMsg(const long long timestamp,
                                    const cv::Mat &img) {}

void DataWriter::WriteSlotMsg(const long long timestamp,
                              const std::vector<Eigen::MatrixXd> &slot_uv) {

  std::string insert_data_dr =
      "INSERT INTO dr_pose (timestamp, data) VALUES (?, ?);";
  patac_hpp::ParkingSlotList ps_list;
  auto *ps = ps_list.add_parking_slot_list();
  for (size_t i = 0; i < slot_uv.size(); ++i) {
    for (size_t j = 0; j < 4; ++j) {
      auto *pt = ps->add_points();
      pt->set_x(slot_uv.at(i).col(j).x());
      pt->set_y(slot_uv.at(i).col(j).y());
    }
  }

  std::string serialized;
  ps_list.SerializeToString(&serialized);
}

} // namespace apa_slam
