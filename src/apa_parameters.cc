/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/apa_parameters.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 4:41:53 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "apa_parameters.h"

namespace apa_slam {
ApaParameters::ApaParameters() {}

ApaParameters &ApaParameters::GetInstance() {
  static ApaParameters instance;
  return instance;
}

void ApaParameters::printout_parameters() {
  std::cout << "========= Project Parameters: =========\n";
  std::cout << "simulation.slot_mea_noise_x: " << _sim_params.slot_mea_noise_x
            << std::endl;
  std::cout << "simulation.slot_mea_noise_y: " << _sim_params.slot_mea_noise_y
            << std::endl;
  std::cout << "simulation.odo_velocity_noise: "
            << _sim_params.odo_velocity_noise << std::endl;
  std::cout << "simulation.odo_angular_velocity_noise: "
            << _sim_params.odo_angular_velocity_noise << std::endl;
  std::cout << "simulation.perception_sensing_range: "
            << _sim_params.perception_sensing_range << std::endl;

  std::cout << "estimatora_noise_x: " << _est_params.slot_mea_noise_x
            << std::endl;
  std::cout << "estimatora_noise_y: " << _est_params.slot_mea_noise_y
            << std::endl;
  std::cout << "estimatorocity_noise: "
            << _est_params.odo_velocity_noise << std::endl;
  std::cout << "estimatorular_velocity_noise: "
            << _est_params.odo_angular_velocity_noise << std::endl;

  std::cout << "estimator.slot_matching_dist_thresh: "
            << _est_params.slot_matching_dist_thresh << std::endl;
  std::cout << "estimator.slot_matching_angle_thresh: "
            << _est_params.slot_matching_angle_thresh << std::endl;
  std::cout << "estimator.slot_min_tracking_times: "
            << _est_params.slot_min_tracking_times << std::endl;
}

const SimulationParams &ApaParameters::GetSimulationParameters() {
  return _sim_params;
}

const EstimatorParams &ApaParameters::GetEstimatorParamters() {
  return _est_params;
}

bool ApaParameters::LoadParameters(const std::string &json_file) {
  try {
    std::ifstream input(json_file);
    json data = json::parse(input);

    _sim_params.slot_mea_noise_x = data["simulation"]["slot_mea_noise_x"];
    _sim_params.slot_mea_noise_y = data["simulation"]["slot_mea_noise_y"];
    _sim_params.odo_velocity_noise = data["simulation"]["odo_velocity_noise"];
    _sim_params.odo_angular_velocity_noise =
        data["simulation"]["odo_angular_velocity_noise"];
    _sim_params.perception_sensing_range =
        data["simulation"]["perception_sensing_range"];

    _est_params.slot_mea_noise_x = data["estimator"]["slot_mea_noise_x"];
    _est_params.slot_mea_noise_y = data["estimator"]["slot_mea_noise_y"];
    _est_params.odo_velocity_noise = data["estimator"]["odo_velocity_noise"];
    _est_params.odo_angular_velocity_noise =
        data["estimator"]["odo_angular_velocity_noise"];
    _est_params.slot_matching_dist_thresh =
        data["estimator"]["slot_matching_dist_thresh"];
    _est_params.slot_matching_angle_thresh =
        data["estimator"]["slot_matching_angle_thresh"];
    _est_params.slot_min_tracking_times =
        data["estimator"]["slot_min_tracking_times"];

    _est_params.max_tracking_time = data["estimator"]["max_tracking_time"];

  } catch (const std::exception &e) {
    std::cerr << "JSON Error: " << e.what() << std::endl;
    return false;
  }
  printout_parameters();
  return true;
}
}  // namespace apa_slam
