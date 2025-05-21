/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/mea_preprocessor.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Wednesday, May 21st 2025, 1:49:07 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "mea_preprocessor.h"

namespace apa_slam {
MeaPreprocessor::MeaPreprocessor() {}

MeaPreprocessor &MeaPreprocessor::GetInstance() {
  static MeaPreprocessor instance;
  return instance;
}

std::vector<SemanticMea::Ptr> MeaPreprocessor::Preprocess(
    const SensorType &type, const std::vector<SemanticMea::Ptr> &meas) {
  std::vector<SemanticMea::Ptr> mea_processed;
  switch (type) {
    case SensorType::SEMANTIC_TYPE_PARKING_SLOT:
      mea_processed = remove_duplicate_slot_meas(meas);
      break;

    default:
      break;
  }

  return mea_processed;
}

std::vector<SemanticMea::Ptr> MeaPreprocessor::remove_duplicate_slot_meas(
    const std::vector<SemanticMea::Ptr> &meas) {
  const double &thresh = ApaParameters::GetInstance()
                             .GetEstimatorParamters()
                             .duplicate_slot_thresh;

  std::vector<bool> remove(meas.size(), false);

  for (size_t i = 0; i < meas.size(); ++i) {
    if (remove[i]) {
      continue;
    }
    Eigen::Vector2d center_i = (meas.at(i)->GetMeaData().col(0).head(2) +
                                meas.at(i)->GetMeaData().col(1).head(2)) *
                               .5f;
    for (size_t j = i + 1; j < meas.size(); ++j) {
      Eigen::Vector2d center_j = (meas.at(j)->GetMeaData().col(0).head(2) +
                                  meas.at(j)->GetMeaData().col(1).head(2)) *
                                 .5f;
      if ((center_i - center_j).norm() < thresh) {
        remove[i] = true;
        remove[j] = true;
      }
    }
  }

  std::vector<SemanticMea::Ptr> unique_meas;
  for (size_t i = 0; i < meas.size(); ++i) {
    if (!remove[i]) {
      unique_meas.push_back(meas.at(i));
    }
  }

  return unique_meas;
}

}  // namespace apa_slam
