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
  _output_file_name = "./" + oss.str() + ".db";
  // _output_file_name = "/userdata/apatest/" + oss.str() + ".db";
  _rc = sqlite3_open(_output_file_name.c_str(), &_db);
  if (_rc) {
    std::cerr << "Can't open database: " << sqlite3_errmsg(_db) << "\n";
    return;
  }
  // 1. 配置数据库参数（提升写入速度）
  char* errMsg = nullptr;
  if (sqlite3_exec(_db, "PRAGMA synchronous = NORMAL;", nullptr, nullptr, &errMsg) != SQLITE_OK) {
    std::cerr << "Failed to disable synchronous: " << errMsg << std::endl;
    sqlite3_free(errMsg);
  }
  if (sqlite3_exec(_db, "PRAGMA journal_mode = WAL;", nullptr, nullptr, &errMsg) != SQLITE_OK) {
    std::cerr << "Failed to set journal_mode: " << errMsg << std::endl;
    sqlite3_free(errMsg);
  }
  //增大页面大小和缓存,减少磁盘访问频率
  sqlite3_exec(_db, "PRAGMA page_size = 4096;", nullptr, nullptr, &errMsg);  // 默认 1024
  sqlite3_exec(_db, "PRAGMA cache_size = -10000;", nullptr, nullptr, &errMsg);  // 10MB 缓存

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

    std::string create_seg_images_tbl = R"(CREATE TABLE IF NOT EXISTS seg_images (
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);)";

  _rc = sqlite3_exec(_db, create_seg_images_tbl.c_str(), nullptr, nullptr, &_errMsg);

  if (_rc != SQLITE_OK) {
    std::cerr << "create_seg_images_tbl failed SQL error: " << _errMsg << std::endl;
    sqlite3_free(_errMsg);
    sqlite3_close(_db);
    return;
  }

  std::cout << "create_seg_images_tbl Succ\n";

    std::string create_pt_cloud_tbl = R"(CREATE TABLE IF NOT EXISTS pt_cloud (
    timestamp INTEGER NOT NULL,
    data BLOB NOT NULL);)";

  _rc = sqlite3_exec(_db, create_pt_cloud_tbl.c_str(), nullptr, nullptr, &_errMsg);

  if (_rc != SQLITE_OK) {
    std::cerr << "create_pt_cloud_tbl failed SQL error: " << _errMsg << std::endl;
    sqlite3_free(_errMsg);
    sqlite3_close(_db);
    return;
  }

  std::cout << "create_pt_cloud_tbl Succ\n";
}

void DataWriter::WriteDataSeq(const long long timestamp, const DataType &type) {
  if (!_initialized_seq) {
  std::string insert_data_seq =
      "INSERT INTO data_seq (timestamp, type) VALUES (?, ?);";
  sqlite3_prepare_v2(_db, insert_data_seq.c_str(), -1, &_stmt_seq, nullptr);
      _initialized_seq = true;
  }

  sqlite3_bind_int64(_stmt_seq, 1, timestamp);
  sqlite3_bind_int(_stmt_seq, 2, type);

  if (sqlite3_step(_stmt_seq) != SQLITE_DONE) {
    std::cerr << "Insert into data_seq failed: " << sqlite3_errmsg(_db)
              << std::endl;
  }

  sqlite3_reset(_stmt_seq);
}

void DataWriter::Reset() {
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
  if (_stmt_segimgs) {
      sqlite3_finalize(_stmt_segimgs);
  }
  _initialized_seq = false;
  _initialized_dr = false;
  _initialized_slots = false;
  _initialized_imgs = false;
  _initialized_segimgs = false; 
  sqlite3_close(_db);
  Init();
} 

void DataWriter::WriteDrPoseMsg(const long long timestamp,
                                const Eigen::Vector3d &pose) {
  if (!_initialized_dr) {
  std::string insert_data_dr =
      "INSERT INTO dr_pose (timestamp, data) VALUES (?, ?);";
  sqlite3_prepare_v2(_db, insert_data_dr.c_str(), -1, &_stmt_dr, nullptr);
  _initialized_dr = true;
  }

  patac_hpp::DRPose dr_pose;
  dr_pose.set_timestamp(timestamp);
  dr_pose.set_x(pose.x());
  dr_pose.set_y(pose.y());
  dr_pose.set_yaw(pose.z());
  std::string serialized;
  dr_pose.SerializeToString(&serialized);

  sqlite3_bind_int64(_stmt_dr, 1, timestamp);
  sqlite3_bind_blob(_stmt_dr, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(_stmt_dr);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into dr_pose failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  sqlite3_reset(_stmt_dr);
}

void DataWriter::WriteImageList(const long long timestamp,
                                const std::vector<cv::Mat> &imgs) {
  if (!_initialized_imgs) {
  std::string insert_data_img =
      "INSERT INTO fisheye_images (timestamp, data) VALUES (?, ?);";
  sqlite3_prepare_v2(_db, insert_data_img.c_str(), -1, &_stmt_imgs, nullptr);
    _initialized_imgs = true;
  }
  patac_hpp::ImageList img_list;
  img_list.set_timestamp(timestamp);
  img_list.set_num_image(imgs.size());
  for (size_t i = 0; i < imgs.size(); ++i) {
    auto image = img_list.add_image_list();
    image->set_width(imgs.at(i).cols);
    image->set_height(imgs.at(i).rows);
    std::vector<uchar> buffer;
    cv::imencode(".png", imgs.at(i), buffer);
    image->set_data(buffer.data(), buffer.size());
  }

  std::string serialized;
  img_list.SerializeToString(&serialized);
  sqlite3_bind_int64(_stmt_imgs, 1, timestamp);
  sqlite3_bind_blob(_stmt_imgs, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(_stmt_imgs);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into fisheye_images failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  sqlite3_reset(_stmt_imgs);
}
void DataWriter::WriteSegImage(const long long timestamp, const std::vector<cv::Mat> &imgs){

  if (!_initialized_segimgs) {
  std::string insert_data_img =
      "INSERT INTO seg_images (timestamp, data) VALUES (?, ?);";
  sqlite3_prepare_v2(_db, insert_data_img.c_str(), -1, &_stmt_segimgs, nullptr);
    _initialized_segimgs = true;
  }
  patac_hpp::ImageList img_list;
  img_list.set_timestamp(timestamp);
  img_list.set_num_image(imgs.size());
  for (size_t i = 0; i < imgs.size(); ++i) {
    auto image = img_list.add_image_list();
    image->set_width(imgs.at(i).cols);
    image->set_height(imgs.at(i).rows);
    std::vector<uchar> buffer;
    cv::imencode(".png", imgs.at(i), buffer);
    image->set_data(buffer.data(), buffer.size());
  }

  std::string serialized;
  img_list.SerializeToString(&serialized);
  sqlite3_bind_int64(_stmt_segimgs, 1, timestamp);
  sqlite3_bind_blob(_stmt_segimgs, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(_stmt_segimgs);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into seg_images failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  sqlite3_reset(_stmt_segimgs);
}

void DataWriter::WriteSlotMsg(const long long timestamp,
                              const std::vector<Eigen::MatrixXd> &slot_uv) {
  if (!_initialized_slots) {
  std::string insert_data_ps =
      "INSERT INTO slot_list (timestamp, data) VALUES (?, ?);";
  sqlite3_prepare_v2(_db, insert_data_ps.c_str(), -1, &_stmt_slots, nullptr);
    _initialized_slots = true;
  }

  patac_hpp::ParkingSlotList ps_list;
  ps_list.set_timestamp(timestamp);
  ps_list.set_num_parking_slot(slot_uv.size());
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
  sqlite3_bind_int64(_stmt_slots, 1, timestamp);
  sqlite3_bind_blob(_stmt_slots, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(_stmt_slots);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into slot_list failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  sqlite3_reset(_stmt_slots);
}

void DataWriter::WritePTCMsg(const long long timestamp,
                             const std::vector<Eigen::MatrixXd> &ptc,const Eigen::Vector4d &pose)
{
  if (!_initialized_ptc) {
  std::string insert_data_ptc =
      "INSERT INTO pt_cloud (timestamp, data) VALUES (?, ?);";
  sqlite3_prepare_v2(_db, insert_data_ptc.c_str(), -1, &_stmt_ptc, nullptr);
    _initialized_ptc = true;
  }

  patac_hpp::Ptcloud ptc_list;
  ptc_list.set_timestamp(timestamp);
  ptc_list.set_num_pt(ptc.size());
  auto ptcpose = ptc_list.mutable_pose();
  ptcpose->set_x(pose(0));
  ptcpose->set_y(pose(1));
  ptcpose->set_z(pose(2));
  ptcpose->set_yaw(pose(3)); 
  for (size_t i = 0; i < ptc.size(); ++i) {
    auto *pt = ptc_list.add_ptcloud_list();
    const Eigen::MatrixXd& point_matrix = ptc[i];

      pt->set_x(point_matrix(0,0));
      pt->set_y(point_matrix(0,1));
      pt->set_z(point_matrix(0,2));
      pt->set_label(point_matrix(0,3));
    }

  std::string serialized;
  ptc_list.SerializeToString(&serialized);
  sqlite3_bind_int64(_stmt_ptc, 1, timestamp);
  sqlite3_bind_blob(_stmt_ptc, 2, serialized.data(), serialized.size(),
                    SQLITE_TRANSIENT);

  int rc = sqlite3_step(_stmt_ptc);
  if (rc != SQLITE_DONE) {
    std::cerr << "Insert into pt_cloud failed: " << sqlite3_errmsg(_db)
              << std::endl;
  } else {
    // std::cout << "Successfully inserted pose at " << timestamp << std::endl;
  }

  sqlite3_reset(_stmt_ptc);
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
