#include <fstream>
#include <iostream>
#include <thread>

#include "ekf_estimator.h"
#include "interface.h"
#include "odo_measurement.h"
#include "parking_slot_measurement.h"
#include "data_reader.h"
#ifdef ENABLE_OPENGL
#include "visualization/pangolin_viewer.h"
#endif
using namespace apa_slam;

#define RESET_TEST 0

void ResetFunc();

int main(int argc, char** argv) {
  const std::string cfg = argv[1];
  std::string data_file = argv[2];
  // LocalMappingInterface::GetInstance().InitMapping(cfg);

#ifdef ENABLE_OPENGL
  PangolinViewer::Ptr viewer = std::make_shared<PangolinViewer>();
  viewer->Start();
#endif

#if RESET_TEST
  std::thread reset_th(&ResetFunc);
  reset_th.detach();
#endif
  while(true)
  {
    if(vis_meas.startMapping)
    {
      LocalMappingInterface::GetInstance().InitMapping(cfg);
      break;
    }
    else if(vis_meas.startLocalization)
    {
      LocalMappingInterface::GetInstance().InitLocalization(cfg);
      break;
    }
    else
    {

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // LocalMappingInterface::GetInstance().Init(cfg);
  apa_slam::DataReader reader(data_file); //读取数据库文件名
  while (true) {
    LocalMappingInterface::GetInstance().Reset();
    // std::ifstream fin;
    // fin.open(data_file, std::ios::in);
    // std::string line;
    // double t;
    int seq_rc = SQLITE_ROW;
    // if (!vis_meas.startMapping && !vis_meas.startLocalization)
    // {
    //   continue;
    // }
    // std::cout<<"vis_meas.startLocalization: "<<vis_meas.startLocalization<<std::endl;
    // std::cout<<"vis_meas.IsLoadMap: "<<vis_meas.IsLoadMap<<std::endl;
    // if (vis_meas.startLocalization && !vis_meas.IsLoadMap)
    // {
      // LocalMappingInterface::GetInstance().InitLocalization(cfg);
      // MapIO::GetInstance().LoadMapData();
      // SemanticMap::GetInstance().LoadMappingData(SEMANTIC_TYPE_PARKING_SLOT, vis_meas.slot_map_data_filename);
      // vis_meas.IsLoadMap = true;
    // }
    // while (getline(fin, line)) {
    while (true) {
      // std::stringstream ss(line);
      // std::string type;
      // ss >> type;
      auto data_seq = reader.ReadDataSeq(seq_rc);
      if(!data_seq.empty()){
       if(1 == data_seq[0].second){
        patac_hpp::DRPose dr_posestmp;
        reader.QueryDataByTimestamp("dr_pose", data_seq[0].first, dr_posestmp,1);
        Eigen::VectorXd pose = Eigen::VectorXd::Zero(3);
        pose[0] = dr_posestmp.x();
        pose[1] = dr_posestmp.y();
        pose[2] = dr_posestmp.yaw();
        long long tsll = static_cast<long long>(dr_posestmp.timestamp());
        LocalMappingInterface::GetInstance().ProcDrPose(tsll, pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
       }else if(2 == data_seq[0].second){
        patac_hpp::ParkingSlotList slots;
        reader.QueryDataByTimestamp("slot_list", data_seq[0].first, slots,2);
        long long tsll = static_cast<long long>(data_seq[0].first);
        std::vector<Eigen::VectorXd> slot_data;
        std::vector<ParkingSlotAttribute> slot_attribute;
        for(int i=0;i<slots.parking_slot_list_size();i++){
          Eigen::MatrixXd mea_data = Eigen::MatrixXd::Zero(2, 2);
          int type=0, parkabel=1;
          Eigen::VectorXd uv = Eigen::VectorXd::Zero(4);
          uv[0] = slots.parking_slot_list(i).points(0).x();
          uv[1] = slots.parking_slot_list(i).points(0).y();
          uv[2] = slots.parking_slot_list(i).points(1).x();
          uv[3] = slots.parking_slot_list(i).points(1).y();
          using namespace std; 
          cout << "uv[0]: " << uv[0] << endl;
          cout << "uv[1]: " << uv[1] << endl;
          cout << "uv[2]: " << uv[2] << endl;
          cout << "uv[3]: " << uv[3] << endl;
          slot_data.push_back(uv);
          apa_slam::ParkingSlotAttribute attr;
          attr.parkable = parkabel;
          attr.slot_type = static_cast<apa_slam::ParkingSlotType>(type);
          slot_attribute.push_back(attr);
        }        
        LocalMappingInterface::GetInstance().ProcSlotData(tsll, slot_data,
                                                          slot_attribute);
        std::cout << "Update Size: " << slot_data.size() << std::endl;

        // LocalMappingInterface::ProcSlotData()

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
       }//else if() (set_target_id){
        // int tar_id;
        // ss >> tar_id;
        // LocalMappingInterface::GetInstance().SetTargetSlotId(tar_id);
        // LocalMappingInterface::GetInstance().NotifyTargetStatus();
        // }
      }
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
