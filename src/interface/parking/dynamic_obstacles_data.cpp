//

#include "interface/parking/dynamic_obstacles_data.h"

#include <cstring>

namespace autodrive {

DynamicObstaclesData::DynamicObstaclesData() noexcept
    : CommonMsgData{}, dynamic_obstacles_{} {}

DynamicObstaclesData::DynamicObstaclesData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, dynamic_obstacles_{} {}

DynamicObstaclesData::DynamicObstacles const&
DynamicObstaclesData::GetDynamicObstacles() const noexcept {
  return dynamic_obstacles_;
}

DynamicObstaclesData::DynamicObstacles&
DynamicObstaclesData::MutableDynamicObstacles() noexcept {
  return dynamic_obstacles_;
}

void DynamicObstaclesData::SetDynamicObstacles(
    DynamicObstacles const& dynamic_obstacle_data) noexcept {
  dynamic_obstacles_ = dynamic_obstacle_data;
}

void DynamicObstaclesData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&dynamic_obstacles_, 0, sizeof(dynamic_obstacles_));
}

}  // namespace autodrive