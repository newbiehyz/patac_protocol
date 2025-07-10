//
#include "interface/parking/map_display_data.h"

#include <cstring>

namespace autodrive {

MapDisplayData::MapDisplayData() noexcept : CommonMsgData{}, map_display_{} {}

MapDisplayData::MapDisplayData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, map_display_{} {}

MapDisplayData::MapDisplay const& MapDisplayData::GetMapDisplayData()
    const noexcept {
  return map_display_;
}

MapDisplayData::MapDisplay& MapDisplayData::MutableMapDisplayData() noexcept {
  return map_display_;
}

void MapDisplayData::SetMapDisplayData(MapDisplay const& map_display) noexcept {
  map_display_ = map_display;
}

void MapDisplayData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&map_display_, 0, sizeof(map_display_));
}

}  // namespace autodrive