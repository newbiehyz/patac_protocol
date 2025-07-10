//
#include "interface/parking/vision_system_data.h"

#include <cstring>

namespace autodrive {

VisionSystemData::VisionSystemData() noexcept
    : CommonMsgData{}, vision_system_data_{0} {}

VisionSystemData::VisionSystemData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, vision_system_data_{} {}

VisionSystemData::VisionSystem const& VisionSystemData::GetVisionSystem()
    const noexcept {
  return vision_system_data_;
}

VisionSystemData::VisionSystem&
VisionSystemData::MutableVisionSystem() noexcept {
  return vision_system_data_;
}

void VisionSystemData::SetVisionSystem(
    VisionSystem const& vision_system_data) noexcept {
  vision_system_data_ = vision_system_data;
}

void VisionSystemData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&vision_system_data_, 0, sizeof(vision_system_data_));
}

}  // namespace autodrive