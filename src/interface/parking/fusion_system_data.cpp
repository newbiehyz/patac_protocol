//
#include "interface/parking/fusion_system_data.h"

#include <cstring>

namespace autodrive {

FusionSystemData::FusionSystemData() noexcept
    : CommonMsgData{}, fusion_system_data_{} {}

FusionSystemData::FusionSystemData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, fusion_system_data_{} {}

FusionSystemData::FusionSystem const& FusionSystemData::GetFusionSystem()
    const noexcept {
  return fusion_system_data_;
}

FusionSystemData::FusionSystem&
FusionSystemData::MutableFusionSystem() noexcept {
  return fusion_system_data_;
}

void FusionSystemData::SetFusionSystem(
    FusionSystem const& fusion_system_data) noexcept {
  fusion_system_data_ = fusion_system_data;
}

void FusionSystemData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&fusion_system_data_, 0, sizeof(fusion_system_data_));
}

}  // namespace autodrive