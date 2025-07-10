//
#include "interface/parking/lane_line_data.h"

#include <cstring>

namespace autodrive {

LaneLineData::LaneLineData() noexcept : CommonMsgData{}, lane_line_points_{} {}

LaneLineData::LaneLineData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, lane_line_points_{} {}

LaneLineData::LaneLinePoints const&
  LaneLineData::GetLaneLinePoints() const noexcept
{
  return lane_line_points_;
}

LaneLineData::LaneLinePoints&
  LaneLineData::MutableLaneLinePoints() noexcept
{ 
  return lane_line_points_; 
}

void LaneLineData::SetLaneLinePoints(
  LaneLinePoints const& lane_line_points) noexcept
{
  lane_line_points_ = lane_line_points;
}

void LaneLineData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&lane_line_points_, 0, sizeof(lane_line_points_));
}

}  // namespace autodrive