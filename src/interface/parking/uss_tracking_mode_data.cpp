//
#include "interface/parking/uss_tracking_mode_data.h"

#include <cstring>

namespace autodrive {

UssTrackingModeData::UssTrackingModeData() noexcept
    : CommonMsgData{}, uss_tracking_mode_data_{} {}

UssTrackingModeData::UssTrackingModeData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, uss_tracking_mode_data_{} {}

UssTrackingModeData::UssTrackingMode const&
UssTrackingModeData::GetUssTrackingMode() const noexcept {
  return uss_tracking_mode_data_;
}

UssTrackingModeData::UssTrackingMode&
UssTrackingModeData::MutableUssTrackingMode() noexcept {
  return uss_tracking_mode_data_;
}

void UssTrackingModeData::SetUssTrackingMode(
    UssTrackingMode const& uss_tracking_mode) noexcept {
  uss_tracking_mode_data_ = uss_tracking_mode;
}

void UssTrackingModeData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&uss_tracking_mode_data_, 0, sizeof(uss_tracking_mode_data_));
}

}  // namespace autodrive