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
  const double l = 5.0f;

  glPointSize(8.0);

  glBegin(GL_POINTS);
  glColor3f(.0f, 1.0f, 0.0f);
  glVertex3f(data.col(0).x(), data.col(0).y(), .0f);
  glVertex3f(data.col(1).x(), data.col(1).y(), .0f);
  glEnd();

  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  Eigen::Vector2d pt = (data.col(0).head(2) + data.col(1).head(2)) * .5f;
  glVertex3f(pt.x(), pt.y(), .0f);
  glEnd();

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

void PangolinDrawer::DrawAPA() {
  if (EkfEstimator::GetInstance().Initialized()) {
    const Pose& latest_pose = EkfEstimator::GetInstance().GetLatestPose();
    const Eigen::MatrixXd latest_cov =
        EkfEstimator::GetInstance().GetLatestCovariance();

    double timestamp = EkfEstimator::GetInstance().GetLatestTimestamp();
    this->draw_vehicle(latest_pose, latest_cov);
    this->draw_traj();
    _traj.insert({timestamp, latest_pose});

    // if (_traj.size() > 1 &&
    //     fabs(_traj.begin()->first - _traj.rbegin()->first) > 5.0) {
    //   _traj.erase(_traj.begin());
    // }
  }

  draw_local_map();
}

void PangolinDrawer::draw_local_map() {
  if (!SemanticMap::GetInstance().HasMap(SEMANTIC_TYPE_PARKING_SLOT)) {
    return;
  }
  const auto slot_map =
      SemanticMap::GetInstance().GetMap(SEMANTIC_TYPE_PARKING_SLOT);
  for (auto it = slot_map.begin(); it != slot_map.end(); ++it) {
    if (it->second->Initialized()) {

      Eigen::MatrixXd data = it->second->GetLandmarkData();
      int id = it->second->GetId();
      draw_parking_slot(id, data);
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
