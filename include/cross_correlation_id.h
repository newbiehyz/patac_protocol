/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/cross_correlation_id.h Path:
 * /home/yukan/Documents/work/local_mapping/include Created Date: Sunday, May
 * 25th 2025, 12:31:17 pm Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once

#include <cstdint>
#include <map>
#include <tuple>


namespace apa_slam {
struct CrossCorrelationId {
  uint8_t type;
  int id;
  CrossCorrelationId() = default;
  CrossCorrelationId(uint8_t t, int i) : type(t), id(i) {}

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

}  // namespace apa_slam

namespace std {
template <>
struct hash<apa_slam::CrossCorrelationId> {
  size_t operator()(const apa_slam::CrossCorrelationId &k) const {
    return hash<int>()(static_cast<int>(k.type)) ^ hash<int>()(k.id);
  }
};

template <>
struct hash<apa_slam::CrossCorrelationKey> {
  size_t operator()(const apa_slam::CrossCorrelationKey &k) const {
    return hash<apa_slam::CrossCorrelationId>()(k.first) ^
           hash<apa_slam::CrossCorrelationId>()(k.second);
  }
};
}  // namespace std
