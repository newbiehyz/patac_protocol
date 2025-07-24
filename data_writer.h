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
#include <sqlite3.h>

#include "patac_dr.pb.h"
#include "patac_image.pb.h"
#include "patac_slot.pb.h"

namespace apa_slam {

enum DataType : std::uint8_t { Pose = 0, DR = 1, SLOT = 2, FISH_FRONT = 3 };

class DataWriter {
public:
  typedef std::shared_ptr<DataWriter> Ptr;

  DataWriter();

  static DataWriter &GetInstance();

  void Init();

  void Reset();

  void WriteDrPoseMsg(const long long timestamp, const Eigen::Vector3d &pose);

  void WriteFrontImageMsg(const long long timestamp, const cv::Mat &img);
  void WriteFrontImageMsg(const long long timestamp, const class patac_hpp::ImageList &img_list);

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
};
} // namespace apa_slam
