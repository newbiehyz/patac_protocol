/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/ekf_management.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Sunday, May 11th 2025, 6:50:19 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>

#include "apa_parameters.h"
#include "local_mapping_define.h"
#include "semantic_map.h"
struct CrossCorrelationId {
  apa_slam::SensorType type;
  int id;
  CrossCorrelationId() = default;
  CrossCorrelationId(apa_slam::SensorType t, int i) : type(t), id(i) {}

  bool operator==(const CrossCorrelationId &other) const {
    return type == other.type && id == other.id;
  }
  bool operator<(const CrossCorrelationId &other) const {
    return std::tie(type, id) < std::tie(other.type, other.id);
  }
};

struct CrossCorrelationKey {
  CrossCorrelationId first;
  CrossCorrelationId second;
  CrossCorrelationKey() = default;
  CrossCorrelationKey(CrossCorrelationId f, CrossCorrelationId s)
      : first(f), second(s) {}

  bool operator==(const CrossCorrelationKey &other) const {
    auto [a1, b1] = std::minmax(first, second);
    auto [a2, b2] = std::minmax(other.first, other.second);
    return a1 == a2 && b1 == b2;
  }
};

namespace std {
template <>
struct hash<CrossCorrelationId> {
  size_t operator()(const CrossCorrelationId &k) const {
    return hash<int>()(static_cast<int>(k.type)) ^ hash<int>()(k.id);
  }
};

template <>
struct hash<CrossCorrelationKey> {
  size_t operator()(const CrossCorrelationKey &k) const {
    return hash<CrossCorrelationId>()(k.first) ^
           hash<CrossCorrelationId>()(k.second);
  }
};
}  // namespace std

namespace apa_slam {

class EKFManagement {
 public:
  typedef std::shared_ptr<EKFManagement> Ptr;
  EKFManagement();
  void Init();
  static EKFManagement &GetInstance();
  void Propagate(const double v, const double w, const double dt,
                 const Eigen::VectorXd &x_vehicle0,
                 const Eigen::MatrixXd &P_vehicle0, Eigen::VectorXd &x_vehicle1,
                 Eigen::MatrixXd &P_vehicle1);

  void Update(const Eigen::VectorXd &state_mean, const Eigen::MatrixXd &state_P, const double timestamp);
  void ClearList();
  void ClearStateList();

 private:
  void state_augmentation(const std::unordered_map<SensorType, std::vector<int>> &augmentation_list);
  void state_marginalization(const std::unordered_map<SensorType, std::vector<int>> &marginalization_list);
  void ekf_update(
      const std::unordered_map<SensorType, std::vector<int>> &update_list);
  CrossCorrelationKey make_lm_cross_correlation_key(const SensorType &type0,
                                                    const int &id0,
                                                    const SensorType &type1,
                                                    const int &id1);
  Eigen::MatrixXd construct_P();

  std::unordered_map<CrossCorrelationKey, Eigen::MatrixXd>
      _lm_cross_correlation;
  std::unordered_map<CrossCorrelationId, Eigen::MatrixXd>
      _state_lm_cross_correlation;

  Eigen::MatrixXd _N;  // odo measurement

  std::unordered_map<SensorType, std::vector<int>> _state_list;  // full state

  Eigen::VectorXd _vehicle_state;
  Eigen::MatrixXd _vehicle_cov;
  double _ts;

  bool _initialized{false};
};
}  // namespace apa_slam
