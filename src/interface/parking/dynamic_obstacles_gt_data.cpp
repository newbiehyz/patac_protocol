//

#include "interface/parking/dynamic_obstacles_gt_data.h"

#include <cstring>

namespace autodrive {

DynamicObstaclesGtData::DynamicObstaclesGtData() noexcept
    : CommonMsgData{}, dynamic_obstacles_{} {}

DynamicObstaclesGtData::DynamicObstaclesGtData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, dynamic_obstacles_{} {}

DynamicObstaclesGtData::DynamicObstacles const&
DynamicObstaclesGtData::GetDynamicObstacles() const noexcept {
  return dynamic_obstacles_;
}

DynamicObstaclesGtData::DynamicObstacles&
DynamicObstaclesGtData::MutableDynamicObstacles() noexcept {
  return dynamic_obstacles_;
}

void DynamicObstaclesGtData::SetDynamicObstacles(
    DynamicObstacles const& dynamic_obstacle_data) noexcept {
  dynamic_obstacles_ = dynamic_obstacle_data;
}

void DynamicObstaclesGtData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&dynamic_obstacles_, 0, sizeof(dynamic_obstacles_));
}

}  // namespace autodrive