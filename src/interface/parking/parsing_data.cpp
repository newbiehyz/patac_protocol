//
#include "interface/parking/parsing_data.h"

#include <cstring>

namespace autodrive {

ParsingData::ParsingData() noexcept : CommonMsgData{}, parsing_data_{} {}

ParsingData::ParsingData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, parsing_data_{} {}

ParsingData::Parsing const& ParsingData::GetParsing() const noexcept {
  return parsing_data_;
}

ParsingData::Parsing& ParsingData::MutableParsing() noexcept {
  return parsing_data_;
}

void ParsingData::SetParsing(Parsing const& parsing_data) noexcept {
  parsing_data_ = parsing_data;
}

void ParsingData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&parsing_data_, 0, sizeof(parsing_data_));
}

}  // namespace autodrive