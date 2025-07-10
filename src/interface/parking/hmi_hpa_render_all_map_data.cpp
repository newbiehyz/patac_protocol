//

#include "interface/parking/hmi_hpa_render_all_map_data.h"

#include <cstring>

namespace autodrive {

HmiHpaRenderAllMapData::HmiHpaRenderAllMapData() noexcept
    : CommonMsgData{}, hmi_hpa_render_{} {}

HmiHpaRenderAllMapData::HmiHpaRenderAllMapData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, hmi_hpa_render_{} {}

HmiHpaRenderAllMapData::HmiHpaAllMapRender const &
HmiHpaRenderAllMapData::GetHmiHpaAllMapRender() const noexcept {
  return hmi_hpa_render_;
}

HmiHpaRenderAllMapData::HmiHpaAllMapRender &
HmiHpaRenderAllMapData::MutableHmiHpaAllMapRender() noexcept {
  return hmi_hpa_render_;
}

void HmiHpaRenderAllMapData::SetHmiHpaAllMapRender(
    HmiHpaAllMapRender const &hmi_hpa_render) noexcept {
  hmi_hpa_render_ = hmi_hpa_render;
}

void HmiHpaRenderAllMapData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&hmi_hpa_render_, 0, sizeof(hmi_hpa_render_));
}

} // namespace autodrive