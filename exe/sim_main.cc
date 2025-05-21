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
#include "sim_data_loader.h"
#ifdef ENABLE_OPENGL
#include "visualization/pangolin_viewer.h"
#endif

using namespace apa_slam;

int main(int argc, char** argv) {
  std::string cfg_path = argv[1];
  std::string dataset_path = argv[2];

  auto& apa_params = ApaParameters::GetInstance();
  apa_params.LoadParameters(cfg_path);

  SimDataLoader::Ptr loader = std::make_shared<SimDataLoader>();
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

      estimator.InputKinematicMea(timestamp, kinematic_meas);
    }

    if (type == ReplaySensorType::REPLAY_TYPE_SEMANTIC) {
      auto semantic_meas = loader->GetSemanticMeas(timestamp);
      estimator.InputSemanticMea(timestamp, semantic_meas);

      std::cout << "=============SEMANTIC================ " << timestamp
                << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (true) {
  }

  return 0;
}