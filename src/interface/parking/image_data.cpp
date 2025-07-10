//
#include "interface/parking/image_data.h"

#include <cstring>

namespace autodrive {

ImageData::ImageData() noexcept : CommonMsgData{}, image_data_{} {}

ImageData::ImageData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, image_data_{} {}

ImageData::Image const& ImageData::GetImageData() const noexcept {
  return image_data_;
}

ImageData::Image& ImageData::MutableImageData() noexcept { return image_data_; }

void ImageData::SetImageData(Image const& image_data) noexcept {
  image_data_ = image_data;
}

void ImageData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&image_data_, 0, sizeof(image_data_));
}

}  // namespace autodrive
