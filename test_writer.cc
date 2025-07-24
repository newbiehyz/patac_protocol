#include "data_writer.h"
#include <Eigen/Eigen>
using namespace apa_slam;

cv::Mat generate_random_shape_image(int width = 640, int height = 480) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255)); // 白底
  int num_shapes = rand() % 10 + 5;

  for (int i = 0; i < num_shapes; ++i) {
    cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
    int thickness = rand() % 3 + 1;

    int shape_type = rand() % 3;
    if (shape_type == 0) { // 圆
      cv::Point center(rand() % width, rand() % height);
      int radius = rand() % 50 + 10;
      cv::circle(img, center, radius, color, thickness);
    } else if (shape_type == 1) { // 矩形
      cv::Point pt1(rand() % width, rand() % height);
      cv::Point pt2(rand() % width, rand() % height);
      cv::rectangle(img, pt1, pt2, color, thickness);
    } else { // 线
      cv::Point pt1(rand() % width, rand() % height);
      cv::Point pt2(rand() % width, rand() % height);
      cv::line(img, pt1, pt2, color, thickness);
    }
  }

  return img;
}

int main() {
  DataWriter::GetInstance().Init();

  long long timestmap = 1e16;

  for (size_t i = 0; i < 10; ++i) {
    int data_type = i % 4;
    DataWriter::GetInstance().WriteDataSeq(
        timestmap, static_cast<apa_slam::DataType>(data_type));
    timestmap += 200;
  }

  for (size_t i = 0; i < 10; ++i) {
    Eigen::Vector3d random_pose = Eigen::Vector3d::Random() * 10.0;
    DataWriter::GetInstance().WriteDrPoseMsg(timestmap, random_pose);
    std::cout << random_pose.transpose()<< std::endl;
    timestmap += 200;
  }

  for (size_t i = 0; i < 10; ++i) {

    cv::Mat random_img = generate_random_shape_image();
    patac_hpp::ImageList imgs_list;
    imgs_list.set_num_image(4); 
    imgs_list.set_timestamp(timestmap);
    // 添加第一个图片
    patac_hpp::Image* img1 = imgs_list.add_image_list();
    img1->set_width(random_img.cols);
    img1->set_height(random_img.rows);
    // img1->set_pos(pos);
    // img1->set_type(type);
    // img1->set_mode(mode);

    // 直接存储BGR原始数据
    const uchar* bgr_data = random_img.ptr<uchar>(0);
    int data_size = random_img.total() * random_img.elemSize();  // 计算数据大小: width * height * 3

    // 将cv::Mat编码为PNG内存数据
    std::vector<uchar> png_buffer;
    cv::imencode(".png", random_img, png_buffer);

    // 设置二进制数据
    // img1->set_data(bgr_data, data_size);//bgr原始
    img1->set_data(png_buffer.data(), png_buffer.size());//png编码

    DataWriter::GetInstance().WriteFrontImageMsg(timestmap, imgs_list);
    timestmap += 200;
    cv::imshow("random", random_img);
    cv::imwrite("output.png", random_img);  // JPEG格式（有损压缩，文件小）
    cv::waitKey(0);
  }

  int cnt = 0;
  for (size_t i = 0; i < 10; ++i) {
    patac_hpp::ParkingSlotList set_ps_list;
    set_ps_list.set_num_parking_slot(1); 
    // 添加第一个停车位
    patac_hpp::ParkingSlot* slot1 = set_ps_list.add_parking_slot_list();
    slot1->set_id(i+1);
    slot1->set_valid(1);
    slot1->set_conf(0.95f); 
    slot1->set_planning_score(1);
    slot1->set_source(patac_hpp::ParkingSource::ParkingSourceIpm);
    slot1->set_lock_status(patac_hpp::ParkingLockStatus::ParkingLockStatusUnknown);
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

    DataWriter::GetInstance().WriteSlotMsg(timestmap, set_ps_list);
    // std::cout << random_pose.transpose()<< std::endl;
    timestmap += 200;
  }

  return 0;
}