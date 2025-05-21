/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/mea_preprocessor.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Wednesday, May 21st 2025, 1:43:03 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <set>
#include <vector>

#include "semantic_measurement.h"

namespace apa_slam {
class MeaPreprocessor {
 public:
  typedef std::shared_ptr<MeaPreprocessor> Ptr;
  MeaPreprocessor();
  static MeaPreprocessor& GetInstance();
  std::vector<SemanticMea::Ptr> Preprocess(
      const SensorType &type, const std::vector<SemanticMea::Ptr> &meas);

 private:
  std::vector<SemanticMea::Ptr> remove_duplicate_slot_meas(
      const std::vector<SemanticMea::Ptr> &meas);
};
}  // namespace apa_slam
