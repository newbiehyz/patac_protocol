//
#include "interface/parking/sync_cylinder_parsing_data.h"

#include <cstring>

namespace autodrive {

SyncCylinderParsingData::SyncCylinderParsingData() noexcept
    : CommonMsgData{}, sync_cylinder_parsing_data_{} {}

SyncCylinderParsingData::SyncCylinderParsingData(
    std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, sync_cylinder_parsing_data_{} {}

SyncCylinderParsingData::SyncCylinderParsing const&
SyncCylinderParsingData::GetSyncCylinderParsing() const noexcept {
  return sync_cylinder_parsing_data_;
}

SyncCylinderParsingData::SyncCylinderParsing&
SyncCylinderParsingData::MutableSyncCylinderParsing() noexcept {
  return sync_cylinder_parsing_data_;
}

void SyncCylinderParsingData::SetSyncCylinderParsing(
    SyncCylinderParsing const& sync_cylinder_parsing_data) noexcept {
  sync_cylinder_parsing_data_ = sync_cylinder_parsing_data;
}

void SyncCylinderParsingData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&sync_cylinder_parsing_data_, 0, sizeof(sync_cylinder_parsing_data_));
}

}  // namespace autodrive