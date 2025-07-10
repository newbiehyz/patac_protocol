//
#include "interface/parking/map_loc_system_data.h"

#include <cstring>

namespace autodrive {

MapLocSystemData::MapLocSystemData() noexcept
    : CommonMsgData{}, map_loc_system_data_{} {}

MapLocSystemData::MapLocSystemData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, map_loc_system_data_{} {}

MapLocSystemData::MapLocSystem const& MapLocSystemData::GetMapLocSystem()
    const noexcept {
  return map_loc_system_data_;
}

MapLocSystemData::MapLocSystem&
MapLocSystemData::MutableMapLocSystem() noexcept {
  return map_loc_system_data_;
}

void MapLocSystemData::SetMapLocSystem(
    MapLocSystem const& map_loc_system_data) noexcept {
  map_loc_system_data_ = map_loc_system_data;
}

void MapLocSystemData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&map_loc_system_data_, 0, sizeof(map_loc_system_data_));
}

}  // namespace autodrive