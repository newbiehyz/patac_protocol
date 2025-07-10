//

#include <cstring>

#include "interface/parking/hmi_render_data.h"

namespace autodrive {

HmiRenderData::HmiRenderData() noexcept
    : CommonMsgData{}, hmi_render_{} {}

HmiRenderData::HmiRenderData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, hmi_render_{} {}

HmiRenderData::HmiRender const& HmiRenderData::GetHmiRender()
    const noexcept {
  return hmi_render_;
}

HmiRenderData::HmiRender&
HmiRenderData::MutableHmiRender() noexcept {
  return hmi_render_;
}

void HmiRenderData::SetHmiRender(
    HmiRender const& hmi_render) noexcept {
  hmi_render_ = hmi_render;
}

void HmiRenderData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&hmi_render_, 0, sizeof(hmi_render_));
}

}  // namespace autodrive