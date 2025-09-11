#include "loc_output.h"

namespace apa_slam {
LocOutput::LocOutput() {}

LocOutput &LocOutput::GetInstance() {
  static LocOutput instance;
  return instance;
}

double LocOutput::angle_diff(double angle1, double angle2) {
  double diff = angle1 - angle2;
  while (diff > M_PI) diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return diff;
}

bool LocOutput::Propagate(Eigen::VectorXd &pos, long long timestamp) {
  auto it = _dr_buf.lower_bound(timestamp);
  while (it != _dr_buf.end()) {
    auto next_it = std::next(it);
    if (next_it != _dr_buf.end()) {
      double dt =
          static_cast<double>(next_it->first - it->first) *
          ApaParameters::GetInstance().GetEstimatorParamters().time_scale;
      double v = it->second.velocity;
      double w = it->second.angular_velocity;
      Eigen::Vector2d twb = pos.head(2);
      double yaw = pos[2];
      Eigen::Rotation2Dd rot(yaw);
      Eigen::Matrix2d Rwb = rot.toRotationMatrix();
      Eigen::Vector2d dir = Rwb.col(0);
      twb += dir * v * dt;
      yaw += dt * w;
      pos.head(2) = twb;
      pos[2] = yaw;
    }
    it++;
  }
  return true;
}

bool LocOutput::GetLatestVechilePos(Eigen::VectorXd &pose) {
  std::cout << "[!!!!!!!!!!!!!!!GetLatestVechilePos]" << std::endl;
  if (!EkfEstimator::GetInstance().Initialized()) {
    return false;
  }
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  long long timestamp;
  std::cout << "[48]" << std::endl;
  if (EkfEstimator::GetInstance().GetLatestVechileState(timestamp, x, P)) {
    std::cout << "x: " << x.transpose() << std::endl;
    if (Propagate(x, timestamp)) {
      pose = x;
      return true;
    }
    return false;
  }
  return false;
}

void LocOutput::CleanOlderData(double current_ts) {
  const double max_duration = 5.0f;
  if (!_dr_buf.empty()) {
    auto it = _dr_buf.begin();
    while (it != _dr_buf.end()) {
      double delta_time =
          fabs(static_cast<double>(current_ts - it->first)) *
          ApaParameters::GetInstance().GetEstimatorParamters().time_scale;
      if (delta_time > max_duration) {
        it = _dr_buf.erase(it);
      } else {
        ++it;
      }
    }
  }
}

bool LocOutput::ProcDrPose(long long ts, const Eigen::VectorXd &pose) {
  apa_slam::Pose p;
  p.x = pose.x();
  p.y = pose.y();
  p.yaw = pose.z();

  double v_out, w_out;
  long long ts_out;
  int size_dr = _dr_pose.size();
  if (size_dr >= 1 && std::fabs(_dr_timestamp[size_dr - 1] - ts) < 1e-4) {
    return false;
  }
  if (_dr_pose.size() < 3) {
    _dr_pose.push_back(p);
    _dr_timestamp.push_back(ts);
    return false;
  } else {
    double ts0 = _dr_timestamp[0];
    double ts1 = _dr_timestamp[2];

    Eigen::Vector2d twb0(_dr_pose[0].x, _dr_pose[0].y);
    Eigen::Vector2d twb1(_dr_pose[2].x, _dr_pose[2].y);
    Eigen::Matrix3d Rwb0 =
        Eigen::AngleAxisd(_dr_pose[0].yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(_dr_pose[2].yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();

    double diff = angle_diff(_dr_pose[2].yaw, _dr_pose[0].yaw);

    double dt = static_cast<double>(ts1 - ts0) *
                ApaParameters::GetInstance().GetEstimatorParamters().time_scale;

    w_out = diff / dt;
    v_out = (twb0 - twb1).norm() / dt;
    ts_out = _dr_timestamp[1];

    Eigen::Vector2d dir0 = Rwb0.col(0).head(2);
    Eigen::Vector2d dir = (twb1 - twb0).normalized();
    if (dir.dot(dir0) < 0) {
      v_out *= -1;
    }

    std::swap(_dr_timestamp[0], _dr_timestamp[1]);
    std::swap(_dr_timestamp[1], _dr_timestamp[2]);

    std::swap(_dr_pose[0], _dr_pose[1]);
    std::swap(_dr_pose[1], _dr_pose[2]);

    _dr_pose[2] = p;
    _dr_timestamp[2] = ts;

    DrInfo dr_info;
    dr_info.pose = _dr_pose[0];
    dr_info.angular_velocity = w_out;
    dr_info.velocity = v_out;
    _dr_buf.insert({ts_out, dr_info});

    CleanOlderData(ts_out);

    return true;
  }
}

void LocOutput::Reset() {
  _dr_buf.clear();
  _dr_pose.clear();
  _dr_timestamp.clear();
}
}  // namespace apa_slam