#include "interface/parking/lane_multitask_data.h"

#include <cstring>
namespace autodrive {

LaneMultitaskData::LaneMultitaskData() noexcept
    : CommonMsgData{},
      camera_id_{0},
      lane_transition_point_{},
      num_lane_parsing_image_{0},
      lane_parsing_image_{0},
      num_lane_contour_{0},
      lane_contours_{0},
      roi_{0} {}

LaneMultitaskData::LaneMultitaskData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp} {
  LaneMultitaskData();
}

std::array<LaneMultitaskData::LaneParsingImageData,
           LaneMultitaskData::kMaxParsingSize> const&
LaneMultitaskData::GetFeatureMap() const noexcept {
  return lane_parsing_image_;
}

std::array<LaneMultitaskData::LaneParsingImageData,
           LaneMultitaskData::kMaxParsingSize>&
LaneMultitaskData::MutableFeatureMap() noexcept {
  return lane_parsing_image_;
}

void LaneMultitaskData::SetFeatureMap(
    std::array<LaneMultitaskData::LaneParsingImageData,
               LaneMultitaskData::kMaxParsingSize> const&
        lane_parsing_image) noexcept {
  lane_parsing_image_ = lane_parsing_image;
}

std::array<RleCompressedData, LaneMultitaskData::kMaxParsingSize> const&
LaneMultitaskData::GetRleFeatureMap()
      const noexcept {
  return rle_lane_parsing_image_;
}

std::array<RleCompressedData, LaneMultitaskData::kMaxParsingSize>&
LaneMultitaskData::MutableRleFeatureMap() noexcept {
  return rle_lane_parsing_image_;
}

LaneMultitaskData::LaneTransitionPoint const&
LaneMultitaskData::GetTransitionPoint() const noexcept {
  return lane_transition_point_;
}

LaneMultitaskData::LaneTransitionPoint&
LaneMultitaskData::MutableTransitionPoint() noexcept {
  return lane_transition_point_;
}

void LaneMultitaskData::SetTransitionPoints(
    LaneTransitionPoint const& lane_transition_point) noexcept {
  lane_transition_point_ = lane_transition_point;
}

std::array<LaneMultitaskData::LaneContour,
           LaneMultitaskData::kMaxLaneCountoursNum>&
LaneMultitaskData::MutableLaneContours() noexcept {
  return lane_contours_;
}

std::uint8_t LaneMultitaskData::GetLaneParsingImageNum() const noexcept {
  return num_lane_parsing_image_;
}

void LaneMultitaskData::SetLaneParsingImageNum(std::uint8_t num) noexcept {
  num_lane_parsing_image_ = num;
}

std::int32_t LaneMultitaskData::GetCameraId() const noexcept {
  return camera_id_;
}

void LaneMultitaskData::SetCameraId(std::int32_t camera_id) noexcept {
  camera_id_ = camera_id;
}

std::uint8_t LaneMultitaskData::GetLaneContourNum() const noexcept {
  return num_lane_contour_;
}

void LaneMultitaskData::SetLaneContourNum(std::uint8_t num) noexcept {
  num_lane_contour_ = num;
}

std::array<LaneMultitaskData::LaneContour,
           LaneMultitaskData::kMaxLaneCountoursNum> const&
LaneMultitaskData::GetLaneContours() noexcept {
  return lane_contours_;
}

Rect const& LaneMultitaskData::GetRoi() const noexcept { return roi_; }

Rect& LaneMultitaskData::MutableRoi() noexcept { return roi_; }

void LaneMultitaskData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  camera_id_ = 0;
  memset(&lane_transition_point_, 0, sizeof(lane_transition_point_));
  num_lane_parsing_image_ = 0;
  memset(&lane_parsing_image_, 0, sizeof(lane_parsing_image_));
  memset(&rle_lane_parsing_image_, 0, sizeof(rle_lane_parsing_image_));
  num_lane_contour_ = 0;
  memset(&lane_contours_, 0, sizeof(lane_contours_));
  memset(&roi_, 0, sizeof(roi_));
}
}  // namespace autodrive
