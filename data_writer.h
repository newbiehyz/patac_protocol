/*
 * Filename: /home/yukan/Documents/work/patac_protocol/data_writer.h
 * Path: /home/yukan/Documents/work/patac_protocol
 * Created Date: Monday, July 14th 2025, 7:11:21 pm
 * Author: yukan
 *
 * Copyright (c) 2025 Your Company
 */

#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/opencv.hpp>
#include "sqlite3.h"

#include "patac_dr.pb.h"
#include "patac_image.pb.h"
#include "patac_slot.pb.h"
#include "data_type.h"
namespace apa_slam {

class DataWriter {
public:
  typedef std::shared_ptr<DataWriter> Ptr;

  DataWriter();

  static DataWriter &GetInstance();

  void Init();

  void Reset();

  void WriteDrPoseMsg(const long long timestamp, const Eigen::Vector3d &pose);

  void WriteImageList(const long long timestamp, const std::vector<cv::Mat> &imgs);

  void WriteDataSeq(const long long timestamp, const DataType &type);

  void WriteSlotMsg(const long long timestamp,
                    const std::vector<Eigen::MatrixXd> &slot_uv);
  void WriteSlotMsg(const long long timestamp,
                    const class patac_hpp::ParkingSlotList &slot_uv);
private:
  void create_tables();
  sqlite3 *_db;
  char *_errMsg{0};
  std::string _output_file_name;
  int _rc;
  sqlite3_stmt* _stmt_imgs = nullptr;  // 保存图片查询状态
  bool _initialized_imgs = false;      // 标记图片是否已初始化查询
  sqlite3_stmt* _stmt_seq = nullptr;  // 保存seq查询状态
  bool _initialized_seq = false;      // 标记seq是否已初始化查询
  sqlite3_stmt* _stmt_dr = nullptr;  // 保存dr片查询状态
  bool _initialized_dr = false;      // 标记dr是否已初始化查询
  sqlite3_stmt* _stmt_slots = nullptr;  // 保存车位查询状态
  bool _initialized_slots = false;      // 标记车位是否已初始化查询
};
} // namespace apa_slam
