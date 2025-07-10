
#include "interface/parking/hmi_hpa_render_data.h"

#include <cstring>

namespace autodrive {

HmiHpaRenderData::HmiHpaRenderData() noexcept
    : CommonMsgData{}, hmi_hpa_render_{} {}

HmiHpaRenderData::HmiHpaRenderData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, hmi_hpa_render_{} {}

HmiHpaRenderData::HmiHpaRender const &
HmiHpaRenderData::GetHmiHpaRender() const noexcept {

    return hmi_hpa_render_;
}

HmiHpaRenderData::HmiHpaRender &
HmiHpaRenderData::MutableHmiHpaRender() noexcept {
  return hmi_hpa_render_;
}

void HmiHpaRenderData::SetHmiHpaRender(
    HmiHpaRender const &hmi_hpa_render) noexcept {
  hmi_hpa_render_ = hmi_hpa_render;
}

void HmiHpaRenderData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&hmi_hpa_render_, 0, sizeof(hmi_hpa_render_));
}

} // namespace autodrive