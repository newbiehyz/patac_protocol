#include <pangolin/pangolin.h>

#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
void DrawGrid(float grid_size = 1.0f, int grid_num = 10) {
  glLineWidth(1.0f);
  glColor3f(0.7f, 0.7f, 0.7f);  // 灰色

  glBegin(GL_LINES);
  for (int i = -grid_num; i <= grid_num; ++i) {
    // Lines parallel to X axis
    glVertex3f(-grid_num * grid_size, i * grid_size, 0);
    glVertex3f(grid_num * grid_size, i * grid_size, 0);

    // Lines parallel to Z axis
    glVertex3f(i * grid_size, -grid_num * grid_size, 0);
    glVertex3f(i * grid_size, grid_num * grid_size, 0);
  }
  glEnd();
}

// 生成 n 个区分度大的 RGB 颜色（避免与黑背景相近）
std::vector<Eigen::Vector3d> generateDistinctOpenGLColors(int n) {
  std::vector<Eigen::Vector3d> colors;
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d c = Eigen::Vector3d::Random();
    colors.push_back(c);
  }

  return colors;
}
double interpolate_angle(const double angle0, const double angle1,
                         const double t) {
  double diff = std::atan2(std::sin(angle1 - angle0),
                           std::cos(angle1 - angle0));  // shortest angle diff
  return angle0 + t * diff;
}

Eigen::Vector2d interpolate_translation(const Eigen::Vector2d &twb0,
                                        const Eigen::Vector2d &twb1,
                                        const double t) {
  return twb0 + t * (twb1 - twb0);
}

int main(int argc, char **argv) {
  std::string dir = argv[1];
  std::string file_local_mapping = dir + "/apa_local_mapping.txt";
  std::map<long long, Eigen::VectorXd> dr_pose;
  std::ifstream fin;
  fin.open(file_local_mapping, std::ios::in);
  std::string line;
  while (getline(fin, line)) {
    std::string header;
    std::stringstream ss(line);
    ss >> header;
    if (header == "pose") {
      double ts, x, y, yaw;
      ss >> ts >> x >> y >> yaw;
      long long tsll = static_cast<long long>(ts * 1e3);
      Eigen::VectorXd dr_data = Eigen::VectorXd::Zero(3);
      dr_data[0] = x;
      dr_data[1] = y;
      dr_data[2] = yaw;
      dr_pose[tsll] = dr_data;
    }
  }
  fin.close();

  double dr_distance = .0f;
  auto it_dr0 = dr_pose.begin();
  auto it_dr1 = it_dr0;
  ++it_dr1;

  while (it_dr1 != dr_pose.end()) {
    dr_distance += (it_dr0->second.head(2) - it_dr1->second.head(2)).norm();
    ++it_dr0;
    ++it_dr1;
  }

  std::string file_mea = dir + "/mea_asso.txt";
  std::map<int, std::map<long long, Eigen::VectorXd>> slot_mea;

  std::fstream fin_mea;
  fin_mea.open(file_mea, std::ios::in);
  while (getline(fin_mea, line)) {
    std::stringstream ss(line);
    std::vector<std::string> result;

    boost::split(result, line, boost::is_any_of(" "));
    int map_id = std::atoll(result.at(0).c_str());
    int n = (result.size() - 1) / 5 - 1;
    if (n < 5) {
      continue;
    }
    for (int i = 0; i < n; ++i) {
      long long ts = std::atoll(result.at(5 * i + 1).c_str());
      double x0 = std::atof(result.at(5 * i + 2).c_str());
      double y0 = std::atof(result.at(5 * i + 3).c_str());
      double x1 = std::atof(result.at(5 * i + 4).c_str());
      double y1 = std::atof(result.at(5 * i + 5).c_str());
      Eigen::VectorXd pt = Eigen::VectorXd::Zero(4);
      pt[0] = x0;
      pt[1] = y0;
      pt[2] = x1;
      pt[3] = y1;
      slot_mea[map_id][ts] = pt;
    }
  }
  fin_mea.close();

  std::cout << "Travel Distance: " << dr_distance << " meter\n";
  double error = dr_pose.rbegin()->second.head(2).norm();
  std::cout << "Error Percent: " << (error / dr_distance) * 100.0 << "%\n";

  std::map<int, std::vector<Eigen::VectorXd>> projection;

  for (auto it_map = slot_mea.begin(); it_map != slot_mea.end(); ++it_map) {
    int map_id = it_map->first;
    // std::cout << dr_pose.begin()->first << " " << dr_pose.rbegin()->first
    // << std::endl;
    for (auto it_mea = it_map->second.begin(); it_mea != it_map->second.end();
         ++it_mea) {
      long long ts = it_mea->first;
      if (ts > dr_pose.begin()->first && ts < dr_pose.rbegin()->first) {
        auto it_lower = dr_pose.upper_bound(ts);
        auto it_upper = it_lower;
        --it_lower;

        double ratio = static_cast<double>(ts - it_lower->first) /
                       static_cast<double>(it_upper->first - it_lower->first);

        Eigen::Vector2d translation_lower = it_lower->second.head(2);
        Eigen::Vector2d translation_upper = it_upper->second.head(2);

        double yaw_lower = it_lower->second.z();
        double yaw_upper = it_upper->second.z();

        Eigen::Vector2d twb = interpolate_translation(translation_lower,
                                                      translation_upper, ratio);
        double yaw = interpolate_angle(yaw_lower, yaw_upper, ratio);

        Eigen::Rotation2Dd rot(yaw);
        Eigen::Matrix2d Rwb = rot.toRotationMatrix();

        Eigen::Vector2d pt0_b = it_mea->second.head(2);
        Eigen::Vector2d pt1_b = it_mea->second.tail(2);

        Eigen::Vector2d pt0_w = Rwb * pt0_b + twb;
        Eigen::Vector2d pt1_w = Rwb * pt1_b + twb;

        Eigen::VectorXd ptw = Eigen::VectorXd::Zero(4);
        ptw.head(2) = pt0_w;
        ptw.tail(2) = pt1_w;
        projection[map_id].push_back(ptw);
      }
    }
  }

  for (auto it = projection.begin(); it != projection.end(); ++it) {
    int map_id = it->first;
    int vector_id = std::distance(projection.begin(), it);
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(4);
    for (size_t i = 0; i < it->second.size(); ++i) {
      mean += it->second.at(i);
    }

    mean /= static_cast<double>(it->second.size());
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(4, 4);
    for (size_t i = 0; i < it->second.size(); ++i) {
      cov += (it->second.at(i) - mean) * (it->second.at(i) - mean).transpose();
    }
    cov /= static_cast<double>(it->second.size());
    std::cout << mean.transpose() << std::endl;
    std::cout << cov << std::endl;
    // std::cout << "------------\n";
  }

  pangolin::CreateWindowAndBind("APA Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  int n_color = projection.size();
  auto colors = generateDistinctOpenGLColors(n_color);

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_POINTS);
    for (auto it = dr_pose.begin(); it != dr_pose.end(); ++it) {
      glVertex3f(it->second.x(), it->second.y(), .0f);
    }
    glEnd();

    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (auto it = projection.begin(); it != projection.end(); ++it) {
      int map_id = it->first;
      int vector_id = std::distance(projection.begin(), it);
      glColor3f(colors.at(vector_id)[0], colors.at(vector_id)[1],
                colors.at(vector_id)[2]);
      for (size_t i = 0; i < it->second.size(); ++i) {
        glVertex3f(it->second.at(i)[0], it->second.at(i)[1], .0f);
        glVertex3f(it->second.at(i)[2], it->second.at(i)[3], .0f);
      }
    }
    glEnd();

    DrawGrid(0.5, 1000);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    pangolin::FinishFrame();
  }

  return 0;
}