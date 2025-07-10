//
#include "foundation/type/common_msg_data.h"

namespace autodrive {

CommonMsgData::CommonMsgData() noexcept : TimeSeriesData{0L}, frame_id_{0L} {}

CommonMsgData::CommonMsgData(std::int64_t timestamp) noexcept :
    TimeSeriesData{timestamp}, frame_id_{0L} {}

std::int64_t CommonMsgData::GetFrameId() const noexcept {
  return frame_id_;
}

void CommonMsgData::SetFrameId(std::int64_t frame_id) noexcept {
  frame_id_ = frame_id;
}

}