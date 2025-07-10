//
#include "interface/parking/fusion_parking_slots_data.h"
#include <cstring>

namespace autodrive {

FusionParkingSlotsData::FusionParkingSlotsData() noexcept
    : CommonMsgData{}, fusion_parking_slots_data_{} {}

FusionParkingSlotsData::FusionParkingSlotsData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, fusion_parking_slots_data_{} {}

FusionParkingSlotsData::FusionParkingSlots const& FusionParkingSlotsData::GetFusionParkingSlots()
    const noexcept {
  return fusion_parking_slots_data_;
}

FusionParkingSlotsData::FusionParkingSlots& FusionParkingSlotsData::MutableFusionParkingSlots() noexcept {
  return fusion_parking_slots_data_;
}

void FusionParkingSlotsData::SetFusionParkingSlots(FusionParkingSlots const& fusion_parking_slots_data) noexcept {
  fusion_parking_slots_data_ = fusion_parking_slots_data;
}

void FusionParkingSlotsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&fusion_parking_slots_data_, 0, sizeof(fusion_parking_slots_data_));
}

}  // namespace autodrive