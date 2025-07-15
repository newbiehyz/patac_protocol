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
    DataWriter::GetInstance().WriteFrontImageMsg(timestmap, random_img);
    timestmap += 200;
    cv::imshow("random", random_img);
    cv::waitKey(0);
  }

  return 0;
}