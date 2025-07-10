//
#include "interface/parking/parking_slots_data.h"

#include <cstring>

namespace autodrive {

ParkingSlotsData::ParkingSlotsData() noexcept
    : CommonMsgData{}, parking_slots_data_{} {}

ParkingSlotsData::ParkingSlotsData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, parking_slots_data_{} {}

ParkingSlotsData::ParkingSlots const& ParkingSlotsData::GetParkingSlots()
    const noexcept {
  return parking_slots_data_;
}

ParkingSlotsData::ParkingSlots&
ParkingSlotsData::MutableParkingSlots() noexcept {
  return parking_slots_data_;
}

void ParkingSlotsData::SetParkingSlots(
    ParkingSlots const& parking_slots_data) noexcept {
  parking_slots_data_ = parking_slots_data;
}

void ParkingSlotsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&parking_slots_data_, 0, sizeof(parking_slots_data_));
}

}  // namespace autodrive