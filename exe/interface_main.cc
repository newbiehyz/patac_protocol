#include <fstream>
#include <iostream>
#include <thread>

#include "ekf_estimator.h"
#include "interface.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#ifdef ENABLE_OPENGL
#include "visualization/pangolin_viewer.h"
#endif
using namespace apa_slam;

#define RESET_TEST 0

void ResetFunc();

int main(int argc, char** argv) {
  const std::string cfg = argv[1];
  std::string data_file = argv[2];

  LocalMappingInterface::GetInstance().Init(cfg);

#ifdef ENABLE_OPENGL
  PangolinViewer::Ptr viewer = std::make_shared<PangolinViewer>();
  viewer->Start();
#endif

#if RESET_TEST
  std::thread reset_th(&ResetFunc);
  reset_th.detach();
#endif

  LocalMappingInterface::GetInstance().Init(cfg);

  while (true) {
    LocalMappingInterface::GetInstance().Reset();
    std::ifstream fin;
    fin.open(data_file, std::ios::in);
    std::string line;
    double t;
    while (getline(fin, line)) {
      std::stringstream ss(line);
      std::string type;
      ss >> type;

      if (type == "pose") {
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
        std::vector<Eigen::VectorXd> slot_data;
        std::vector<ParkingSlotAttribute> slot_attribute;
        long long tsll = static_cast<long long>(ts * 1000);
        // std::vector<SemanticMea::Ptr> mea_vector;
        for (int i = 0; i < n; ++i) {
          Eigen::MatrixXd mea_data = Eigen::MatrixXd::Zero(2, 2);
          double u0, v0, u1, v1;
          int type, parkabel;
          ss >> u0 >> v0 >> u1 >> v1 >> type >> parkabel;
          Eigen::VectorXd uv = Eigen::VectorXd::Zero(4);
          uv[0] = u0;
          uv[1] = v0;
          uv[2] = u1;
          uv[3] = v1;

          slot_data.push_back(uv);
          apa_slam::ParkingSlotAttribute attr;
          slot_attribute.push_back(attr);
        }
        LocalMappingInterface::GetInstance().ProcSlotData(tsll, slot_data,
                                                          slot_attribute);
        std::cout << "Update Size: " << slot_data.size() << std::endl;

        // LocalMappingInterface::ProcSlotData()

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }


      if (type == "set_target_id") {
        int tar_id;
        ss >> tar_id;
        LocalMappingInterface::GetInstance().SetTargetSlotId(tar_id);
      }
    }
    fin.close();

    while(true) {
      
    }
  }

  return 0;
}

void ResetFunc() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(15, 25);  // 随机间隔 15 到 25 秒

  while (true) {
    int waitSeconds = dist(gen);
    std::this_thread::sleep_for(std::chrono::seconds(waitSeconds));
    LocalMappingInterface::GetInstance().Reset();
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
  }
}
