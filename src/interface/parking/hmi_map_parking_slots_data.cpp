//

#include "interface/parking/hmi_map_parking_slots_data.h"

#include <cstring>

namespace autodrive {

HmiMapParkingSlotsData::HmiMapParkingSlotsData() noexcept
    : CommonMsgData{}, map_parking_slots_{} {}

HmiMapParkingSlotsData::HmiMapParkingSlotsData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, map_parking_slots_{} {}

HmiMapParkingSlotsData::MapParkingSlots const &
HmiMapParkingSlotsData::GetMapParkingSlots() const noexcept {
  return map_parking_slots_;
}

HmiMapParkingSlotsData::MapParkingSlots&
HmiMapParkingSlotsData::MutableMapParkingSlots() noexcept {
  return map_parking_slots_;
}

void HmiMapParkingSlotsData::SetMapParkingSlots(
    HmiMapParkingSlotsData::MapParkingSlots const &map_parking_slots) noexcept {
  map_parking_slots_ = map_parking_slots;
}

void HmiMapParkingSlotsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&map_parking_slots_, 0, sizeof(map_parking_slots_));
}

} // namespace autodrive