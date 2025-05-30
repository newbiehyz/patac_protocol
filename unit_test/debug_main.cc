#include <pangolin/pangolin.h>

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

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
  std::string debug_file = argv[1];

  std::map<double, Eigen::VectorXd> dr_pose_data;
  std::map<double, Eigen::VectorXd> kinematic_data;
  std::map<double, std::vector<Eigen::MatrixXd>> slot_data;

  std::ifstream fin;
  fin.open(debug_file, std::ios::in);

  std::string line;
  while (getline(fin, line)) {
    std::stringstream ss(line);
    std::string type;
    ss >> type;

    if (type == "pose") {
      double ts, x, y, yaw;
      ss >> ts >> x >> y >> yaw;
      Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);
      pose[0] = x;
      pose[1] = y;
      pose[2] = yaw;
      dr_pose_data[ts] = pose;
    }

    if (type == "odo") {
      double ts, v, w;
      ss >> ts >> v >> w;
      Eigen::VectorXd odo = Eigen::VectorXd::Zero(2);
      odo[0] = v;
      odo[1] = w;
      kinematic_data[ts] = odo;
    }

    if (type == "slot") {
      double ts;
      int n;
      ss >> ts >> n;

      for (int i = 0; i < n; ++i) {
        Eigen::MatrixXd slot = Eigen::MatrixXd::Zero(2, 2);
        double x0, x1, y0, y1;
        ss >> x0 >> y0 >> x1 >> y1;
        slot(0, 0) = x0;
        slot(1, 0) = y0;
        slot(0, 1) = x1;
        slot(1, 1) = y1;
        slot_data[ts].push_back(slot);
      }
    }
  }

  std::map<double, Eigen::VectorXd> propagate_pose;
  Eigen::Vector2d twb0(0, 0);
  double yaw0(0);

  std::map<double, std::vector<Eigen::Vector2d>> projection_dr,
      projection_propagate;

  auto it_kinematic0 = kinematic_data.begin();
  auto it_kinematic1 = it_kinematic0;
  ++it_kinematic1;

  while (it_kinematic1 != kinematic_data.end()) {
    double dt = it_kinematic1->first - it_kinematic0->first;
    double v = it_kinematic0->second.x();
    double w = it_kinematic0->second.y();

    Eigen::Rotation2Dd rot(yaw0);
    Eigen::Matrix2d Rwb = rot.toRotationMatrix();
    Eigen::Vector2d dir = Rwb.col(0);

    twb0 += dir * v * dt;
    yaw0 += dt * w;

    Eigen::VectorXd pose0 = Eigen::VectorXd::Zero(3);
    pose0.head(2) = twb0;
    pose0[2] = yaw0;
    propagate_pose[it_kinematic1->first] = pose0;
    ++it_kinematic0;
    ++it_kinematic1;
  }

  for (auto it = slot_data.begin(); it != slot_data.end(); ++it) {
    if (it->first < propagate_pose.begin()->first ||
        it->first > propagate_pose.rbegin()->first) {
      continue;
    }

    auto it_upper = propagate_pose.upper_bound(it->first);
    auto it_lower = it_upper;
    --it_lower;

    double ratio =
        (it->first - it_lower->first) / (it_upper->first - it_lower->first);

    double yaw_dr =
        interpolate_angle(it_lower->second.z(), it_upper->second.z(), ratio);
    Eigen::Vector2d twb_lower_dr = it_lower->second.head(2);
    Eigen::Vector2d twb_upper_dr = it_upper->second.head(2);

    Eigen::Vector2d twb_dr =
        interpolate_translation(twb_lower_dr, twb_upper_dr, ratio);

    Eigen::Rotation2Dd rot_dr(yaw_dr);
    Eigen::Matrix2d Rwb_dr = rot_dr.toRotationMatrix();

    for (int i = 0; i < it->second.size(); ++i) {
      Eigen::Vector2d pt0 = Rwb_dr * it->second.at(i).col(0).head(2) + twb_dr;
      Eigen::Vector2d pt1 = Rwb_dr * it->second.at(i).col(1).head(2) + twb_dr;
      projection_propagate[it->first].push_back(pt0);
      projection_propagate[it->first].push_back(pt1);
    }
  }

  for (auto it = slot_data.begin(); it != slot_data.end(); ++it) {
    if (it->first < kinematic_data.begin()->first ||
        it->first > kinematic_data.rbegin()->first) {
      continue;
    }

    auto it_upper_dr = dr_pose_data.upper_bound(it->first);
    auto it_lower_dr = it_upper_dr;
    --it_lower_dr;

    double ratio = (it->first - it_lower_dr->first) /
                   (it_upper_dr->first - it_lower_dr->first);

    double yaw_dr = interpolate_angle(it_lower_dr->second.z(),
                                      it_upper_dr->second.z(), ratio);
    Eigen::Vector2d twb_lower_dr = it_lower_dr->second.head(2);
    Eigen::Vector2d twb_upper_dr = it_upper_dr->second.head(2);

    Eigen::Vector2d twb_dr =
        interpolate_translation(twb_lower_dr, twb_upper_dr, ratio);

    Eigen::Rotation2Dd rot_dr(yaw_dr);
    Eigen::Matrix2d Rwb_dr = rot_dr.toRotationMatrix();

    for (int i = 0; i < it->second.size(); ++i) {
      Eigen::Vector2d pt0 = Rwb_dr * it->second.at(i).col(0).head(2) + twb_dr;
      Eigen::Vector2d pt1 = Rwb_dr * it->second.at(i).col(1).head(2) + twb_dr;
      projection_dr[it->first].push_back(pt0);
      projection_dr[it->first].push_back(pt1);
    }
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

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    glBegin(GL_POINTS);
    glPointSize(2.0);
    glColor3f(1.0, 1.0, 1.0);

    for (auto it = projection_propagate.begin();
         it != projection_propagate.end(); ++it) {
      for (size_t i = 0; i < it->second.size(); ++i) {
        glVertex3f(it->second.at(i).x() + 20.0, it->second.at(i).y(), .0f);
      }
    }

    glEnd();

    glBegin(GL_POINTS);
    glPointSize(2.0);
    glColor3f(0.0, 1.0, 1.0);

    for (auto it = projection_dr.begin(); it != projection_dr.end(); ++it) {
      for (size_t i = 0; i < it->second.size(); ++i) {
        glVertex3f(it->second.at(i).x(), it->second.at(i).y(), .0f);
      }
    }

    glEnd();

    DrawGrid(0.5f, 100);

    pangolin::FinishFrame();
  }

  return 0;
}