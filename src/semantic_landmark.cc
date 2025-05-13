/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/semantic_landmark.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Thursday, May 8th 2025, 6:53:08 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include "semantic_landmark.h"

namespace apa_slam {
SemanticLandmark::SemanticLandmark(const SensorType& type, const int id)
    : _type(type), _id(id), _initialized(false), _margin(false) {}

SensorType SemanticLandmark::GetSemanticType() { return _type; }

int SemanticLandmark::GetId() { return _id; }

bool SemanticLandmark::Initialized() { return _initialized; }

bool SemanticLandmark::Margin() { return _margin; }

void SemanticLandmark::NotifyAugmentation() {
  EKFManagement::GetInstance().AddAugmentationList(this->GetSemanticType(),
                                                   this->GetId());
}



void SemanticLandmark::NotifyUpdate() {}

void SemanticLandmark::NotifyMarginalization() {}

}  // namespace apa_slam
