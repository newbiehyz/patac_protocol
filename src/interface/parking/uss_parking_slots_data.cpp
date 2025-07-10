//
#include "interface/parking/uss_parking_slots_data.h"

#include <cstring>

namespace autodrive {

UssParkingSlotsData::UssParkingSlotsData() noexcept
    : CommonMsgData{}, uss_parking_slots_{} {}

UssParkingSlotsData::UssParkingSlotsData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, uss_parking_slots_{} {}

UssParkingSlotsData::UssParkingSlots const&
UssParkingSlotsData::GetUssParkingSlots() const noexcept {
  return uss_parking_slots_;
}

UssParkingSlotsData::UssParkingSlots&
UssParkingSlotsData::MutableUssParkingSlots() noexcept {
  return uss_parking_slots_;
}

void UssParkingSlotsData::SetUssParkingSlots(
    UssParkingSlots const& uss_parking_slots) noexcept {
  uss_parking_slots_ = uss_parking_slots;
  SetFrameId(uss_parking_slots_.frame_id);
  SetRawTimestamp(uss_parking_slots_.sensor_stamp);
}

void UssParkingSlotsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&uss_parking_slots_, 0, sizeof(uss_parking_slots_));
}

}  // namespace autodrive