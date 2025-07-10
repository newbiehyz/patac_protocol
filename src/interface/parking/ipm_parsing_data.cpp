//
#include "interface/parking/ipm_parsing_data.h"

#include <cstring>

namespace autodrive {

IpmParsingData::IpmParsingData() noexcept
    : CommonMsgData{}, ipm_parsing_data_{} {}

IpmParsingData::IpmParsingData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, ipm_parsing_data_{} {}

IpmParsingData::IpmParsing const& IpmParsingData::GetIpmParsing()
    const noexcept {
  return ipm_parsing_data_;
}

IpmParsingData::IpmParsing& IpmParsingData::MutableIpmParsing() noexcept {
  return ipm_parsing_data_;
}

void IpmParsingData::SetIpmParsing(
    IpmParsing const& ipm_parsing_data) noexcept {
  ipm_parsing_data_ = ipm_parsing_data;
}

void IpmParsingData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&ipm_parsing_data_, 0, sizeof(ipm_parsing_data_));
}

}  // namespace autodrive