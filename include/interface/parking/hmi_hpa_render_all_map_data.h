
#ifndef AUTODRIVE_INTERFACE_HMI_HPA_ENGINE_ALL_MAP_DATA_H_
#define AUTODRIVE_INTERFACE_HMI_HPA_ENGINE_ALL_MAP_DATA_H_

#include <cstdint>

#include "interface/parking/hmi_map_display_data.h"

namespace autodrive {

class HmiHpaRenderAllMapData final : public CommonMsgData {
public:
  struct alignas(8) HmiMapInfo {
    /// @brief map common header
    HmiMapDisplayData::MapCommonHeader header;
    /// @brief distance of already mapping (m)
    float map_distance;
    /// @brief rest distance of allow to do mapping (m)
    float rest_map_distance;
    /// @brief num of speed bumps in map
    std::uint8_t num_speed_bumps;
    std::uint8_t padding[7];
  };

  struct alignas(8) HmiAllMapDisplay {
    /// @brief Map info, only used when update_map_info is true
    HmiMapInfo map_info;
    std::uint8_t padding[1];
    /// @brief Choose update the map info or map headers
    bool update_map_info;
    /// @brief Num of maps, not used when update_map_info is true
    std::int8_t num_maps;
    /// @brief the target parking slot Id matches new Id
    bool is_match;
    /// @brief the target map id
    std::int32_t map_id;
    /// @brief All map headers, not used when update_map_info is true
    std::array<HmiMapDisplayData::MapCommonHeader, HmiMapDisplayData::kMaxMapSize>
        all_map_headers;
  };

  struct alignas(8) HmiHpaAllMapRender {
    HmiAllMapDisplay hmi_map_display;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit HmiHpaRenderAllMapData(std::int64_t timestamp) noexcept;

  HmiHpaRenderAllMapData() noexcept;
  ~HmiHpaRenderAllMapData() noexcept final = default;
  HmiHpaRenderAllMapData(HmiHpaRenderAllMapData const &) = default;
  HmiHpaRenderAllMapData &operator=(HmiHpaRenderAllMapData const &) = default;
  HmiHpaRenderAllMapData(HmiHpaRenderAllMapData &&) = default;
  HmiHpaRenderAllMapData &operator=(HmiHpaRenderAllMapData &&) = default;

  /// @brief Get Hmi Hpa Render
  ///
  /// @return hmi_hpa_render_
  HmiHpaAllMapRender const &GetHmiHpaAllMapRender() const noexcept;

  /// @brief Mutable Hmi Hpa Render
  ///
  /// @return hmi_hpa_render_
  HmiHpaAllMapRender &MutableHmiHpaAllMapRender() noexcept;

  /// @brief Set Hmi Hpa Render
  ///
  /// @param hmi_render Hmi Hpa render
  void SetHmiHpaAllMapRender(HmiHpaAllMapRender const &hmi_hpa_render) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

private:
  HmiHpaAllMapRender hmi_hpa_render_;
};

} // namespace autodrive

#endif // AUTODRIVE_INTERFACE_HMI_HPA_ENGINE_DATA_H_
