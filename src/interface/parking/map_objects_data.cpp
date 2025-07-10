//
#include "interface/parking/map_objects_data.h"

#include <cstring>

namespace autodrive {

MapObjectsData::MapObjectsData() noexcept : CommonMsgData{}, objects_{} {}

MapObjectsData::MapObjectsData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, objects_{} {}

MapObjectsData::MapObjects const& MapObjectsData::GetMapObjects()
    const noexcept {
  return objects_;
}

MapObjectsData::MapObjects& MapObjectsData::MutableMapObjects() noexcept {
  return objects_;
}

void MapObjectsData::SetMapObjects(MapObjects const& objects) noexcept {
  objects_ = objects;
}

void MapObjectsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&objects_, 0, sizeof(objects_));
}

}  // namespace autodrive