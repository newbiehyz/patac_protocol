/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/apa_parameters.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Thursday, May 8th 2025, 4:14:52 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <fstream>
#include <iostream>
#include <memory>

#include "local_mapping_define.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace apa_slam {
class ApaParameters {
 public:
  ApaParameters();
  static ApaParameters& GetInstance();
  bool LoadParameters(const std::string& json_file);
  const SimulationParams& GetSimulationParameters();

 private:
  void printout_parameters();
  SimulationParams _sim_params;
};
}  // namespace apa_slam
