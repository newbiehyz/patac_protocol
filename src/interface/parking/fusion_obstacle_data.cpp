//
#include "interface/parking/fusion_obstacle_data.h"

#include <cstring>

namespace autodrive {

FusionObstacleData::FusionObstacleData() noexcept
    : CommonMsgData{}, fusion_obstacle_data_{} {}

FusionObstacleData::FusionObstacleData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, fusion_obstacle_data_{} {}

FusionObstacleData::FusionObstacles const& FusionObstacleData::GetFusionObstacle()
    const noexcept {
  return fusion_obstacle_data_;
}

FusionObstacleData::FusionObstacles&
FusionObstacleData::MutableFusionObstacle() noexcept {
  return fusion_obstacle_data_;
}

void FusionObstacleData::SetFusionObstacle(
    FusionObstacles const& fusion_obstacle_data) noexcept {
  fusion_obstacle_data_ = fusion_obstacle_data;
}

void FusionObstacleData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&fusion_obstacle_data_, 0, sizeof(fusion_obstacle_data_));
}

}  // namespace autodrive