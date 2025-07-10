//
#include "model/camera_data.h"

#include <cstring>

namespace autodrive {

CameraData::CameraData() noexcept : CommonMsgData{}, image_{0} {
}

CameraData::CameraData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp}, image_{0} {
}

CameraData::Image const& CameraData::GetImage() const noexcept {
  return image_;
}

CameraData::Image& CameraData::MutableImage() noexcept {
  return image_;
}

void CameraData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&image_, 0, sizeof(image_));
}

}

/* vim: set ts=2 sw=2 sts=2 tw=100 et: */
