//
#include "model/camera_extrinsics_data.h"

#include <cstring>

namespace autodrive {

CameraExtrinsicsData::CameraExtrinsicsData() noexcept :
    CommonMsgData{}, extrinsics_{0U} {
}

CameraExtrinsicsData::CameraExtrinsicsData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp} {
}

CameraExtrinsicsData::Extrinsics const& CameraExtrinsicsData::GetExtrinsics()
    const noexcept {
  return extrinsics_;
}

CameraExtrinsicsData::Extrinsics& CameraExtrinsicsData::MutableExtrinsics()
    noexcept {
  return extrinsics_;
}

void CameraExtrinsicsData::SetExtrinsics(const Extrinsics& extrinsics) noexcept {
  extrinsics_ = extrinsics;
}

void CameraExtrinsicsData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&extrinsics_, 0, sizeof(extrinsics_));
}

}