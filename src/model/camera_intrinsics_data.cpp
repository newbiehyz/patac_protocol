//
#include "model/camera_intrinsics_data.h"

#include <cstring>

namespace autodrive {

CameraIntrinsicsData::CameraIntrinsicsData() noexcept :
    CommonMsgData{}, intrinsics_{0U} {
}

CameraIntrinsicsData::CameraIntrinsicsData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp} {
}

CameraIntrinsicsData::Intrinsics const& CameraIntrinsicsData::GetIntrinsics()
    const noexcept {
  return intrinsics_;
}

CameraIntrinsicsData::Intrinsics& CameraIntrinsicsData::MutableIntrinsics()
    noexcept {
  return intrinsics_;
}

void CameraIntrinsicsData::SetIntrinsics(const Intrinsics& intrinsics) noexcept {
  intrinsics_ = intrinsics;
}

void CameraIntrinsicsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&intrinsics_, 0, sizeof(intrinsics_));
}

}