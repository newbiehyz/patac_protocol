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
PangolinDrawer::PangolinDrawer() {}

void PangolinDrawer::draw_traj() {
  glColor3f(1.0, 1.0, 1.0);
  glPointSize(3.0);
  glBegin(GL_POINTS);
  for (const auto& pt : _traj) {
    glVertex3f(pt.second.x, pt.second.y, .0);
  }
  glEnd();
}

void PangolinDrawer::draw_parking_slot(const Eigen::MatrixXd& data) {
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
    double timestamp = EkfEstimator::GetInstance().GetLatestTimestamp();
    this->draw_vehicle(latest_pose);
    this->draw_traj();
    _traj.insert({timestamp, latest_pose});

    if (_traj.size() > 1 &&
        fabs(_traj.begin()->first - _traj.rbegin()->first) > 5.0) {
      _traj.erase(_traj.begin());
    }
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
      draw_parking_slot(data);
    }
  }
}

void PangolinDrawer::draw_vehicle(const Pose& latest_pose) {
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
}

}  // namespace apa_slam
