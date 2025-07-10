//
#include "interface/parking/reference_lines_data.h"

#include <cstring>

namespace autodrive {

ReferenceLinesData::ReferenceLinesData() noexcept
    : CommonMsgData{}, reference_lines_{} {}

ReferenceLinesData::ReferenceLinesData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, reference_lines_{} {}

ReferenceLinesData::ReferenceLines const&
ReferenceLinesData::GetReferenceLines() const noexcept {
  return reference_lines_;
}

ReferenceLinesData::ReferenceLines&
ReferenceLinesData::MutableReferenceLines() noexcept {
  return reference_lines_;
}

void ReferenceLinesData::SetReferenceLines(
    ReferenceLines const& reference_lines_data) noexcept {
  reference_lines_ = reference_lines_data;
}

void ReferenceLinesData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(static_cast<void*>(&reference_lines_), 0, sizeof(reference_lines_));
}

}  // namespace autodrive