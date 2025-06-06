/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/visualization/pangolin_drawer.cc
 * Path: /home/yukan/Documents/work/local_mapping/src/visualization
 * Created Date: Friday, May 9th 2025, 11:54:48 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "visualization/pangolin_drawer.h"

namespace apa_slam {
PangolinDrawer::PangolinDrawer() {
  _font = std::make_shared<pangolin::GlFont>(font_ttf, 20);

  int sl_sz = ApaParameters::GetInstance().GetEstimatorParamters().window_size;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> sat_dist(0.7, 1.0);  // 饱和度范围
  std::uniform_real_distribution<> val_dist(0.8, 1.0);  // 亮度范围

  // 黄金角度近似，确保色相均匀分布
  const double golden_ratio = 0.618033988749895;
  double h = 0.0;

  for (int i = 0; i < sl_sz; ++i) {
    h += golden_ratio;
    h = fmod(h, 1.0);

    double s = sat_dist(gen);
    double v = val_dist(gen);

    // HSV 转 RGB
    double r, g, b;
    int hi = static_cast<int>(h * 6);
    double f = h * 6 - hi;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    switch (hi % 6) {
      case 0:
        r = v;
        g = t;
        b = p;
        break;
      case 1:
        r = q;
        g = v;
        b = p;
        break;
      case 2:
        r = p;
        g = v;
        b = t;
        break;
      case 3:
        r = p;
        g = q;
        b = v;
        break;
      case 4:
        r = t;
        g = p;
        b = v;
        break;
      case 5:
        r = v;
        g = p;
        b = q;
        break;
    }

    _sl_color.emplace_back(r, g, b);
  }
}

void PangolinDrawer::draw_traj() {
  glColor3f(1.0, 1.0, 1.0);
  glPointSize(3.0);
  glBegin(GL_POINTS);
  for (const auto& pt : _traj) {
    glVertex3f(pt.second.x, pt.second.y, .0);
  }
  glEnd();
}

void PangolinDrawer::draw_parking_slot(const int& id,
                                       const Eigen::MatrixXd& data) {
  glPointSize(8.0);

  glBegin(GL_POINTS);
  glColor3f(.0f, 1.0f, 0.0f);
  glVertex3f(data.col(0).x(), data.col(0).y(), .0f);
  glVertex3f(data.col(1).x(), data.col(1).y(), .0f);
  glVertex3f(data.col(2).x(), data.col(2).y(), .0f);
  glVertex3f(data.col(3).x(), data.col(3).y(), .0f);
  glEnd();

  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  Eigen::Vector2d pt = (data.col(0).head(2) + data.col(1).head(2)) * .5f;
  glVertex3f(pt.x(), pt.y(), .0f);
  glEnd();

  glColor3f(.0f, .0f, 1.0f);
  glLineWidth(2.0);
  glBegin(GL_LINE_STRIP);

  glVertex3f(data.col(1).x(), data.col(1).y(), .0f);
  glVertex3f(data.col(2).x(), data.col(2).y(), .0f);
  glVertex3f(data.col(3).x(), data.col(3).y(), .0f);
  glVertex3f(data.col(0).x(), data.col(0).y(), .0f);

  glEnd();

  glColor3f(1.0f, 1.0f, 1.0f);
  _font->Text(std::to_string(id)).Draw(0.5 + pt.x(), 0.5 + pt.y());
}

void PangolinDrawer::draw_vehicle_bbox() {
  glColor3f(0.5, 0.0, 0.5);
  glLineWidth(3.0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glEnd();

  glBegin(GL_LINE_STRIP);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glEnd();
}

void PangolinDrawer::draw_sliding_window() {
  SlwVisualization sw;
  // {
  //   std::lock_guard<std::mutex> lock(gl_slw.mutex);
  sw.sl_pose = gl_slw.sl_pose;
  // }
  for (size_t i = 0; i < sw.sl_pose.size(); ++i) {
    Eigen::Matrix3d Rwb =
        Eigen::AngleAxisd(sw.sl_pose.at(i).z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner(3, 3) = Rwb;
    T.topRightCorner(2, 1) = sw.sl_pose.at(i).head(2);

    glPushMatrix();
    glMultMatrixd(T.data());
    glLineWidth(2.0);

    glColor3f(_sl_color.at(i).x(), _sl_color.at(i).y(), _sl_color.at(i).z());
    glBegin(GL_LINE_LOOP);
    glVertex3f(0.4, .0, .0);
    glVertex3f(0.0, -0.4, .0);
    glVertex3f(0.0, 0.4, .0);
    glEnd();

    glPopMatrix();
  }
}

void PangolinDrawer::DrawAPA() {
  std::map<SensorType, std::vector<SemanticMea::Ptr>> cur_meas;
  std::map<SensorType, std::vector<int>> cur_matching;
  if (EkfEstimator::GetInstance().Initialized()) {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    long long ts;
    EkfEstimator::GetInstance().GetLatestVechileState(ts, x, P);
    Pose pose;
    pose.x = x.x();
    pose.y = x.y();
    pose.yaw = x.z();

    this->draw_vehicle(pose, P);
    this->draw_traj();
    _traj.insert({ts, pose});
    draw_local_map(pose);
    draw_sliding_window();
    // draw_local_meas();
  }
}

void PangolinDrawer::draw_local_meas() {
  std::vector<Eigen::VectorXd> slot_meas;

  {
    std::lock_guard<std::mutex> lock(vis_meas.meas_mutex);
    slot_meas = vis_meas.slot_meas;
  }
  glColor3f(1.0f, .0f, 0.0f);
  for (size_t i = 0; i < slot_meas.size(); ++i) {
    pangolin::glDrawCirclePerimeter(slot_meas[i][0], slot_meas[i][1], 0.5);
    pangolin::glDrawCirclePerimeter(slot_meas[i][2], slot_meas[i][3], 0.5);
  }
}

void PangolinDrawer::draw_local_map(const Pose& pose) {
  if (!SemanticMap::GetInstance().HasMap(SEMANTIC_TYPE_PARKING_SLOT)) {
    return;
  }
  const auto& slot_map =
      SemanticMap::GetInstance().GetMap(SEMANTIC_TYPE_PARKING_SLOT);
  for (auto it = slot_map.begin(); it != slot_map.end(); ++it) {
    if (it->second->Initialized()) {
      // Eigen::MatrixXd data = it->second->GetLandmarkData();
      auto slot = std::dynamic_pointer_cast<ParkingSlotLandmark>(it->second);
      Eigen::MatrixXd slot_data = slot->ConstructFullSlot();
      int id = it->second->GetId();
      draw_parking_slot(id, slot_data);
    }
  }
}

void PangolinDrawer::draw_vehicle(const Pose& latest_pose,
                                  const Eigen::MatrixXd& latest_cov) {
  Eigen::Vector3d twb(latest_pose.x, latest_pose.y, .0);
  Eigen::AngleAxisd axang(latest_pose.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d Rwb = axang.toRotationMatrix();
  Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
  Twb.topLeftCorner(3, 3) = Rwb;
  Twb.topRightCorner(3, 1) = twb;

  glPushMatrix();
  glMultMatrixd(Twb.data());
  this->draw_vehicle_bbox();
  glPopMatrix();

  //   Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(
  //       latest_cov.topLeftCorner(2, 2));
  //   Eigen::Vector2d eigenvalues = solver.eigenvalues();
  //   Eigen::Matrix2d eigenvectors = solver.eigenvectors();

  //   double angle =
  //       std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));  // 主方向角度

  //   const int segments = 40;
  //   double a = std::sqrt(eigenvalues(1));  // 长轴（大特征值）
  //   double b = std::sqrt(eigenvalues(0));
  //   glBegin(GL_LINE_LOOP);
  //   for (int i = 0; i < segments; ++i) {
  //     double theta = 2.0 * M_PI * double(i) / double(segments);
  //     double x = a * std::cos(theta);
  //     double y = b * std::sin(theta);

  //     // 旋转 + 平移
  //     double xr = std::cos(angle) * x - std::sin(angle) * y + twb(0);
  //     double yr = std::sin(angle) * x + std::cos(angle) * y + twb(1);

  //     glVertex2d(xr, yr);
  //   }
  //   glEnd();
}

}  // namespace apa_slam
