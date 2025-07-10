//
#include "interface/parking/uss_obstacles_data.h"

#include <cstring>

namespace autodrive {

UssObstaclesData::UssObstaclesData() noexcept
    : CommonMsgData{}, uss_obstacles_{} {}

UssObstaclesData::UssObstaclesData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, uss_obstacles_{} {}

UssObstaclesData::UssObstacles const& UssObstaclesData::GetUssObstaclesData()
    const noexcept {
  return uss_obstacles_;
}

UssObstaclesData::UssObstacles&
UssObstaclesData::MutableUssObstaclesData() noexcept {
  return uss_obstacles_;
}

void UssObstaclesData::SetUssObstaclesData(
    UssObstacles const& uss_obstacles) noexcept {
  uss_obstacles_ = uss_obstacles;
  SetFrameId(uss_obstacles_.frame_id);
  SetRawTimestamp(uss_obstacles_.sensor_stamp);
}

void UssObstaclesData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&uss_obstacles_, 0, sizeof(uss_obstacles_));
}

}  // namespace autodrive