//
#include "interface/parking/static_obstacles_data.h"

#include <cstring>

namespace autodrive {

StaticObstaclesData::StaticObstaclesData() noexcept
    : CommonMsgData{}, static_obstacles_data_{} {}

StaticObstaclesData::StaticObstaclesData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, static_obstacles_data_{} {}

StaticObstaclesData::StaticObstacles const&
StaticObstaclesData::GetStaticObstacles() const noexcept {
  return static_obstacles_data_;
}

StaticObstaclesData::StaticObstacles&
StaticObstaclesData::MutableStaticObstacles() noexcept {
  return static_obstacles_data_;
}

void StaticObstaclesData::SetStaticObstacles(
    StaticObstacles const& static_obstacles_data) noexcept {
  static_obstacles_data_ = static_obstacles_data;
}

void StaticObstaclesData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&static_obstacles_data_, 0, sizeof(static_obstacles_data_));
}

}  // namespace autodrive