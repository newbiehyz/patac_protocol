//
#include "foundation/type/time_series_data.h"

#include <cstdint>

namespace autodrive {

TimeSeriesData::TimeSeriesData() noexcept : local_timestamp_{0L},
    raw_timestamp_{0L} {}

TimeSeriesData::TimeSeriesData(std::int64_t timestamp) noexcept :
    local_timestamp_{timestamp}, raw_timestamp_{0L} {}

std::int64_t TimeSeriesData::GetTimestamp() const noexcept {
  return local_timestamp_;
}

void TimeSeriesData::SetTimestamp(std::int64_t timestamp) noexcept {
  local_timestamp_ = timestamp;
}

std::int64_t TimeSeriesData::GetRawTimestamp() const noexcept {
  return raw_timestamp_;
}

void TimeSeriesData::SetRawTimestamp(std::int64_t timestamp) noexcept {
  raw_timestamp_ = timestamp;
}

}

/* vim: set ts=2 sw=2 sts=2 tw=100 et: */
