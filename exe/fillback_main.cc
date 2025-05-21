/*
 * Filename: /home/yukan/Documents/work/local_mapping/exe/sim_main.cc
 * Path: /home/yukan/Documents/work/local_mapping/exe
 * Created Date: Thursday, May 8th 2025, 2:15:43 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include <iostream>
#include <thread>

#include "apa_parameters.h"
#include "ekf_estimator.h"
#include "fillback_data_loader.h"
#ifdef ENABLE_OPENGL
#include "visualization/pangolin_viewer.h"
#endif

using namespace apa_slam;

int main(int argc, char** argv) {
  std::string cfg_path = argv[1];
  std::string dataset_path = argv[2];

  auto& apa_params = ApaParameters::GetInstance();
  apa_params.LoadParameters(cfg_path);

  FillbackDataLoader::Ptr loader = std::make_shared<FillbackDataLoader>();
  loader->LoadDataSet(dataset_path);

#ifdef ENABLE_OPENGL
  PangolinViewer::Ptr viewer = std::make_shared<PangolinViewer>();
  viewer->Start();
#endif

  auto& estimator = EkfEstimator::GetInstance();
  estimator.Init();

  ReplaySensorType type;
  double timestamp;
  while (loader->PopOutMea(type, timestamp)) {
    if (type == ReplaySensorType::REPLAY_TYPE_KINEMATIC) {
      auto kinematic_meas = loader->GetKinematicMeas(timestamp);

      // std::cout << "=============KINAMATIC================ " <<
      // std::setprecision(20) << timestamp
      // << std::endl;
      estimator.InputKinematicMea(timestamp, kinematic_meas);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (type == ReplaySensorType::REPLAY_TYPE_SEMANTIC) {
      auto semantic_meas = loader->GetSemanticMeas(timestamp);

      std::cout << "=============SEMANTIC================ "
                << semantic_meas.size() << " ----- " << std::setprecision(20)
                << timestamp << std::endl;
      estimator.InputSemanticMea(timestamp, semantic_meas);
      // getchar();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    
  }

  while (true) {
  }

  return 0;
}