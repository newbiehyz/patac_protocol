//
#include "interface/parking/lane_onl_calib_data.h"

#include <cstring>

namespace autodrive {

LaneOnlCalibData::LaneOnlCalibData() noexcept
    : CommonMsgData{}, num_camera_lane_{0}, camera_lane_{0} {}

std::array<LaneOnlCalibData::SingleCamLane,
           LaneOnlCalibData::kMaxCameraNumber> const &
LaneOnlCalibData::GetCameraLane() const noexcept {
  return camera_lane_;
}

std::array<LaneOnlCalibData::SingleCamLane,
           LaneOnlCalibData::kMaxCameraNumber> &
LaneOnlCalibData::MutableCameraLane() noexcept {
  return camera_lane_;
}

void LaneOnlCalibData::SetCameraLane(
    std::array<SingleCamLane, LaneOnlCalibData::kMaxCameraNumber> const
        &camera_lane) noexcept {
  camera_lane_ = camera_lane;
}

void LaneOnlCalibData::SetCameraNum(
    const std::uint8_t &num_camera_lane) noexcept {
  num_camera_lane_ = num_camera_lane;
}

uint8_t const &LaneOnlCalibData::GetCameraNum() const noexcept {
  return num_camera_lane_;
}

uint8_t &LaneOnlCalibData::MutableCameraNum() noexcept {
  return num_camera_lane_;
}

void LaneOnlCalibData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&num_camera_lane_, 0, sizeof(num_camera_lane_));
  memset(&camera_lane_, 0, sizeof(camera_lane_));
}

}  // namespace autodrive
