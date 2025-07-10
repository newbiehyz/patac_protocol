//

#include "interface/parking/hmi_map_display_data.h"

#include <cstring>

namespace autodrive {

HmiMapDisplayData::HmiMapDisplayData() noexcept
    : CommonMsgData{}, map_display_{} {}

HmiMapDisplayData::HmiMapDisplayData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, map_display_{} {}

HmiMapDisplayData::MapDisplay const &
HmiMapDisplayData::GetMapDisplayData() const noexcept {
  return map_display_;
}

HmiMapDisplayData::MapDisplay &
HmiMapDisplayData::MutableMapDisplayData() noexcept {
  return map_display_;
}

void HmiMapDisplayData::SetMapDisplayData(
    MapDisplay const& map_display) noexcept {
  map_display_ = map_display;
}

void HmiMapDisplayData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&map_display_, 0, sizeof(map_display_));
}

} // namespace autodrive