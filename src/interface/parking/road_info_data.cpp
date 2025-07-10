//
#include "interface/parking/road_info_data.h"

#include <cstring>

namespace autodrive {

RoadInfoData::RoadInfoData() noexcept : CommonMsgData{}, road_info_data_{} {}

RoadInfoData::RoadInfoData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, road_info_data_{} {}

RoadInfoData::RoadInfo const& RoadInfoData::GetRoadInfo() const noexcept {
  return road_info_data_;
}

RoadInfoData::RoadInfo& RoadInfoData::MutableRoadInfo() noexcept {
  return road_info_data_;
}

void RoadInfoData::SetRoadInfo(RoadInfo const& road_info_data) noexcept {
  road_info_data_ = road_info_data;
}

std::int32_t RoadInfoData::GetCameraId() const noexcept {
  return camera_id_;
}

void RoadInfoData::SetCameraId(std::int32_t camera_id) noexcept {
  camera_id_ = camera_id;
}

void RoadInfoData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&road_info_data_, 0, sizeof(road_info_data_));
}

}  // namespace autodrive