#include <Eigen/Eigen>

#include "data_writer.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace apa_slam;

cv::Mat generate_random_shape_image(int width = 640, int height = 480) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255));  // 白底
  int num_shapes = rand() % 10 + 5;

  for (int i = 0; i < num_shapes; ++i) {
    cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
    int thickness = rand() % 3 + 1;

    int shape_type = rand() % 3;
    if (shape_type == 0) {  // 圆
      cv::Point center(rand() % width, rand() % height);
      int radius = rand() % 50 + 10;
      cv::circle(img, center, radius, color, thickness);
    } else if (shape_type == 1) {  // 矩形
      cv::Point pt1(rand() % width, rand() % height);
      cv::Point pt2(rand() % width, rand() % height);
      cv::rectangle(img, pt1, pt2, color, thickness);
    } else {  // 线
      cv::Point pt1(rand() % width, rand() % height);
      cv::Point pt2(rand() % width, rand() % height);
      cv::line(img, pt1, pt2, color, thickness);
    }
  }

  return img;
}

int main() {
  DataWriter::GetInstance().Init();

  long long timestamp = 1e16;

  for (size_t i = 0; i < 10; ++i) {


    DataWriter::GetInstance().WriteDataSeq(timestamp,
                                           static_cast<apa_slam::DataType>(1));

    Eigen::Vector3d random_pose = Eigen::Vector3d::Random() * 10.0;

    // 获取当前时间点（系统时钟）
    auto now = std::chrono::system_clock::now();
    // 转换为毫秒时间戳（Unix 时间戳，毫秒级）
    auto millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    DataWriter::GetInstance().WriteDrPoseMsg(timestamp, random_pose);
    // 获取当前时间点（系统时钟）
    auto end_now = std::chrono::system_clock::now();
    // 转换为毫秒时间戳（Unix 时间戳，毫秒级）
    auto end_millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_now.time_since_epoch()
    ).count();
    std::cout<<"dr save time: "<<end_millisec_since_epoch-millisec_since_epoch<<","<<millisec_since_epoch<<","<<end_millisec_since_epoch<<std::endl;
    timestamp += 200;
  }


  for (size_t i = 0; i < 10; ++i) {

    std::vector<cv::Mat> imgs;
    for (size_t j = 0; j < 4; ++j) {
      cv::Mat img = generate_random_shape_image();
      imgs.push_back(img);
    }

    DataWriter::GetInstance().WriteDataSeq(timestamp,
                                           static_cast<apa_slam::DataType>(3));
                                              
    auto startnow = std::chrono::system_clock::now();
    auto millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
    startnow.time_since_epoch()
    ).count();
    DataWriter::GetInstance().WriteImageList(timestamp, imgs);
    auto end_now = std::chrono::system_clock::now();
    auto end_millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_now.time_since_epoch()
    ).count();
    std::cout<<"img save time: "<<end_millisec_since_epoch-millisec_since_epoch<<","<<millisec_since_epoch<<","<<end_millisec_since_epoch<<std::endl;
    timestamp += 200;
    // cv::imshow("random", imgs.at(0));
    // cv::waitKey(1);
  }


  int cnt = 0;
  for (size_t i = 0; i < 10; ++i) {
    patac_hpp::ParkingSlotList set_ps_list;
    set_ps_list.set_num_parking_slot(1);
    // 添加第一个停车位
    patac_hpp::ParkingSlot* slot1 = set_ps_list.add_parking_slot_list();
    slot1->set_id(i + 1);
    slot1->set_valid(1);
    slot1->set_conf(0.95f);
    slot1->set_planning_score(1);
    slot1->set_source(patac_hpp::ParkingSource::ParkingSourceIpm);
    slot1->set_lock_status(
        patac_hpp::ParkingLockStatus::ParkingLockStatusUnknown);
    slot1->set_type(patac_hpp::ParkingSlotType::SlotTypeVertical);
    slot1->set_occupancy(patac_hpp::OccupancyStatus::OccupancyStatusOccupied);

    // 添加4个slot坐标点
    cnt++;
    for (int i = 0; i < 4; ++i) {
      patac_hpp::Point2f point;
      point.set_x(i + 1.0f + cnt);
      point.set_y(i + 1.0f + cnt);
      *slot1->add_points() = point;  // 拷贝赋值
    }
    DataWriter::GetInstance().WriteDataSeq(timestamp,
                                           static_cast<apa_slam::DataType>(2));
    // 获取当前时间点（系统时钟）
    auto now = std::chrono::system_clock::now();
    // 转换为毫秒时间戳（Unix 时间戳，毫秒级）
    auto millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    DataWriter::GetInstance().WriteSlotMsg(timestamp, set_ps_list);
        // 获取当前时间点（系统时钟）
    auto end_now = std::chrono::system_clock::now();
    // 转换为毫秒时间戳（Unix 时间戳，毫秒级）
    auto end_millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_now.time_since_epoch()
    ).count();
    std::cout<<"slots save time: "<<end_millisec_since_epoch-millisec_since_epoch<<","<<millisec_since_epoch<<","<<end_millisec_since_epoch<<std::endl;
    // std::cout << random_pose.transpose()<< std::endl;
    timestamp += 200;
  }

  return 0;
}