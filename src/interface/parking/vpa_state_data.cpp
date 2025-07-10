//
#include "interface/parking/vpa_state_data.h"

#include <cstring>

namespace autodrive {

VpaStateData::VpaStateData() noexcept : CommonMsgData{}, vpa_state_{} {}

VpaStateData::VpaStateData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, vpa_state_{} {}

VpaStateData::VpaState const& VpaStateData::GetVpaStateData() const noexcept {
  return vpa_state_;
}

VpaStateData::VpaState& VpaStateData::MutableVpaStateData() noexcept {
  return vpa_state_;
}

void VpaStateData::SetVpaStateData(VpaState const& vpa_state) noexcept {
  vpa_state_ = vpa_state;
}

void VpaStateData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&vpa_state_, 0, sizeof(vpa_state_));
}

}  // namespace autodrive