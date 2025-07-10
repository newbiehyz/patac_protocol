//

#include "interface/parking/vision_debug_data.h"

#include <cstring>

namespace autodrive {

VisionDebugData::VisionDebugData() noexcept
    : CommonMsgData{}, dynamic_obstacles_debug_info_{} {}

VisionDebugData::VisionDebugData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, dynamic_obstacles_debug_info_{} {}

VisionDebugData::DynamicObstaclesDebugInfo const &
VisionDebugData::GetDynamicObstaclesDebugInfo() const noexcept
{
    return dynamic_obstacles_debug_info_;
}

VisionDebugData::DynamicObstaclesDebugInfo &
VisionDebugData::MutableDynamicObstaclesDebugInfo() noexcept
{
    return dynamic_obstacles_debug_info_;
}

void VisionDebugData::SetDynamicObstaclesDebugInfo(
    DynamicObstaclesDebugInfo const &dynamic_obstacles_debug_info) noexcept
{
    dynamic_obstacles_debug_info_ = dynamic_obstacles_debug_info;
}

void VisionDebugData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&dynamic_obstacles_debug_info_, 0, sizeof(DynamicObstaclesDebugInfo));
}

}  // namespace autodrive