//
#include "interface/parking/speed_bump_data.h"

#include <cstring>

namespace autodrive {

SpeedBumpData::SpeedBumpData() noexcept : CommonMsgData{}, speed_bumps_{} {}

SpeedBumpData::SpeedBumpData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, speed_bumps_{} {}

SpeedBumpData::SpeedBumps const& SpeedBumpData::GetSpeedBumps() const noexcept {
  return speed_bumps_;
}

SpeedBumpData::SpeedBumps& SpeedBumpData::MutableSpeedBumps() noexcept {
  return speed_bumps_;
}

void SpeedBumpData::SetSpeedBumps(SpeedBumps const& speed_bumps) noexcept {
  speed_bumps_ = speed_bumps;
}

void SpeedBumpData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&speed_bumps_, 0, sizeof(speed_bumps_));
}

}  // namespace autodrive