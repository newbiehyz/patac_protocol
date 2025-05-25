/*
 * Filename: /home/yukan/Documents/work/local_mapping/unit_test/test_ekf_key.cc
 * Path: /home/yukan/Documents/work/local_mapping/unit_test
 * Created Date: Monday, May 12th 2025, 9:29:20 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#include <unordered_map>

#include "local_mapping_define.h"
using namespace apa_slam;

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
  CrossCorrelationKey(CrossCorrelationId f, CrossCorrelationId s) {
    if (std::tie(f.type, f.id) < std::tie(s.type, s.id)) {
      first = f;
      second = s;
    } else {
      first = s;
      second = f;
    }
  }

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

CrossCorrelationKey make_lm_cross_correlation_key(const SensorType &type0,
                                                  const int &id0,
                                                  const SensorType &type1,
                                                  const int &id1) {
  auto id_a = CrossCorrelationId{type0, id0};
  auto id_b = CrossCorrelationId{type1, id1};
  return CrossCorrelationKey{id_a, id_b};
}

int main()

{
  std::unordered_map<CrossCorrelationKey, Eigen::MatrixXd> cross_correlation;

  auto key0 =
      make_lm_cross_correlation_key(SensorType::SEMANTIC_TYPE_PARKING_SLOT, 2,
                                    SensorType::SEMANTIC_TYPE_PARKING_SLOT, 5);

  cross_correlation[key0] = Eigen::MatrixXd::Random(5, 4);

  auto key1 =
      make_lm_cross_correlation_key(SensorType::SEMANTIC_TYPE_PARKING_COLUMN, 1,
                                    SensorType::SEMANTIC_TYPE_PARKING_SLOT, 5);
  cross_correlation[key1] = Eigen::MatrixXd::Random(5, 4);

  auto key3 =
      make_lm_cross_correlation_key(SensorType::SEMANTIC_TYPE_PARKING_COLUMN, 1,
                                    SensorType::SEMANTIC_TYPE_PARKING_SLOT, 2);
  cross_correlation[key1] = Eigen::MatrixXd::Random(5, 4);

  return 0;
}