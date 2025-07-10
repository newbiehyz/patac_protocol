//
#include "interface/parking/processor_info_data.h"

#include <cstring>

namespace autodrive {

ProcessorInfoData::ProcessorInfoData() noexcept
    : CommonMsgData{}, processor_info_data_{} {}

ProcessorInfoData::ProcessorInfoData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, processor_info_data_{} {}

ProcessorInfoData::ProcessorInfo const& ProcessorInfoData::GetProcessorInfo()
    const noexcept {
  return processor_info_data_;
}

ProcessorInfoData::ProcessorInfo&
ProcessorInfoData::MutableProcessorInfo() noexcept {
  return processor_info_data_;
}

void ProcessorInfoData::SetProcessorInfo(
    ProcessorInfo const& processor_info) noexcept {
  processor_info_data_ = processor_info;
}

void ProcessorInfoData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&processor_info_data_, 0, sizeof(processor_info_data_));
}

}  // namespace autodrive