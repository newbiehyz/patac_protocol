//
#include "interface/parking/fusion_freespace_data.h"

#include <cstring>

namespace autodrive {

FusionFreespaceData::FusionFreespaceData() noexcept
    : CommonMsgData{}, fusion_freespace_data_{} {}

FusionFreespaceData::FusionFreespaceData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, fusion_freespace_data_{} {}

FusionFreespaceData::FreespaceFusion const& FusionFreespaceData::GetFusionFreespace()
    const noexcept {
  return fusion_freespace_data_;
}

FusionFreespaceData::FreespaceFusion&
FusionFreespaceData::MutableFusionFreespace() noexcept {
  return fusion_freespace_data_;
}

void FusionFreespaceData::SetFusionFreespace(
    FreespaceFusion const& fusion_result_data) noexcept {
  fusion_freespace_data_ = fusion_result_data;
}

void FusionFreespaceData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&fusion_freespace_data_, 0, sizeof(fusion_freespace_data_));
}

}  // namespace autodrive