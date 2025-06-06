#include <fstream>
#include <iostream>

#include "ekf_estimator.h"
#include "interface.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "visualization/pangolin_viewer.h"
using namespace apa_slam;

int main(int argc, char** argv) {
  const std::string cfg = argv[1];
  std::string data_file = argv[2];

  LocalMappingInterface::GetInstance().Init(cfg);

#ifdef ENABLE_OPENGL
  PangolinViewer::Ptr viewer = std::make_shared<PangolinViewer>();
  viewer->Start();
#endif

  std::ifstream fin;
  fin.open(data_file, std::ios::in);
  std::string line;
  double t;
  while (getline(fin, line)) {
    std::stringstream ss(line);
    std::string type;
    ss >> type;

    if (type == "pose") {
      // Eigen::VectorXd mea_data = Eigen::VectorXd::Zero(2);
      // double v, w;
      // double ts;
      // ss >> ts >> v >> w;
      // mea_data[0] = v;
      // mea_data[1] = w;

      // KinematicMea::Ptr odo_mea = std::make_shared<OdoMea>(ts,
      // mea_data.data()); std::vector<KinematicMea::Ptr> mea_vector;
      // mea_vector.push_back(odo_mea);

      Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);
      double ts;
      ss >> ts >> pose[0] >> pose[1] >> pose[2];
      long long tsll = static_cast<long long>(ts * 1000);
      LocalMappingInterface::GetInstance().ProcDrPose(tsll, pose);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (type == "slot") {
      double ts;
      int n;
      ss >> ts >> n;
      long long tsll = static_cast<long long>(ts * 1000);
      std::vector<SemanticMea::Ptr> mea_vector;
      for (int i = 0; i < n; ++i) {
        Eigen::MatrixXd mea_data = Eigen::MatrixXd::Zero(2, 2);
        double x0, y0, x1, y1;
        ss >> x0 >> y0 >> x1 >> y1;
        Eigen::Vector2d pt0(x0, y0);
        Eigen::Vector2d pt1(x1, y1);

        Eigen::Vector2d dir = pt1 - pt0;
        dir.normalize();
        pt0 += 0.12 * dir;
        pt1 -= 0.12 * dir;

        mea_data.col(0) = pt0;
        mea_data.col(1) = pt1;

        if (mea_data.col(0).norm() > ApaParameters::GetInstance()
                                         .GetEstimatorParamters()
                                         .slot_mea_max_range ||
            mea_data.col(1).norm() > ApaParameters::GetInstance()
                                         .GetEstimatorParamters()
                                         .slot_mea_max_range) {
          continue;
        }

        SemanticMea::Ptr mea =
            std::make_shared<ParkingSlotMea>(tsll, mea_data.data());
        mea_vector.push_back(mea);
      }

      EkfEstimator::GetInstance().InputSemanticMea(tsll, mea_vector);
      std::cout << "Update Size: " << mea_vector.size() << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  while (true) {
  }
  return 0;
}