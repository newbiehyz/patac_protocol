//
#include "interface/parking/fusion_result_data.h"

#include <cstring>

namespace autodrive {

FusionResultData::FusionResultData() noexcept
    : CommonMsgData{}, fusion_result_data_{} {}

FusionResultData::FusionResultData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, fusion_result_data_{} {}

FusionResultData::FusionResult const& FusionResultData::GetFusionResult()
    const noexcept {
  return fusion_result_data_;
}

FusionResultData::FusionResult&
FusionResultData::MutableFusionResult() noexcept {
  return fusion_result_data_;
}

void FusionResultData::SetFusionResult(
    FusionResult const& fusion_result_data) noexcept {
  fusion_result_data_ = fusion_result_data;
}

void FusionResultData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&fusion_result_data_, 0, sizeof(fusion_result_data_));
}

}  // namespace autodrive