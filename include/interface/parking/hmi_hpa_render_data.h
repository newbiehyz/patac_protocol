
#ifndef AUTODRIVE_INTERFACE_HMI_HPA_ENGINE_DATA_H_
#define AUTODRIVE_INTERFACE_HMI_HPA_ENGINE_DATA_H_

#include <cstdint>

#include "interface/parking/fusion_obstacle_data.h"
#include "interface/parking/localization_data.h"
#include "interface/parking/map_display_data.h"
#include "interface/parking/vpa_state_data.h"

namespace autodrive {

class HmiHpaRenderData final : public CommonMsgData {
public:
  static constexpr std::uint32_t kMaxHmiGlbPrkgSlotsSeNSize{128U};
  static constexpr std::uint32_t kMaxHmiGlbPrcpObstacleSeNSize{200U};
  struct alignas(8) HmiVpaState {
    /// @brief Should always check this first,only update state variables
    /// relevant to the event type each time
    VpaStateData::VPAEventType vpa_event;
    /// @brief state variables
    VpaStateData::HPAState hpa_state;
    VpaStateData::MappingOddType mapping_odd_type;
    VpaStateData::LocOddType loc_odd_type;
    VpaStateData::MapSavingStage map_saving_stage;
    VpaStateData::LocalLocStage local_loc_stage;
    VpaStateData::MapDeletingStage map_deleted_stage;
    VpaStateData::MappingStage map_stage;
    VpaStateData::MapOptimizeStage map_optimized_stage;
    VpaStateData::ErrorCode error_code;
    std::uint8_t padding[2];
    /// @brief Used  When hpa_state==VPAEventType::kInParkingSlotEvent, if
    /// target_parking_slot_id < 0,means the car not in parking slot else it's
    /// the parking slot id car parking in.
    int32_t target_parking_slot_id;
    /// @brief Used When hpa_state==VPAEventType::kMapSavingStageEvent, [0-100]
    uint32_t map_saving_progress;
    /// @brief Used When hpa_state==VPAEventType::kFindAvailableMapEvent
    std::uint32_t avaliable_map_id;
  };
  struct alignas(8) HmiLocalization {
    /// @brief Header belong to ndm
    NdmSpec ndm_header;
    /// @brief Location info
    LocalizationData::SpLocation location;
  };
  struct alignas(8) HmiGlbPrcpPrkgSlots {
    std::uint32_t num_glb_prkg_slots;
    std::uint8_t padding[4];
    std::array<MapDisplayData::GlbPrkgSlot, kMaxHmiGlbPrkgSlotsSeNSize>
        glb_prkg_slots;
  };

  struct alignas(8) HmiGlbPrcpObstacle {
    std::uint32_t num_glb_prcp_obstacle;
    std::uint8_t padding[4];
    std::array<MapDisplayData::GlbObstacle, kMaxHmiGlbPrcpObstacleSeNSize>
        glb_prcp_obstacle;
  };
  struct alignas(8) HmiMapInfo {
    /// @brief map common header
    MapDisplayData::MapCommonHeader header;
    /// @brief global trajectory
    MapDisplayData::GlbTrajectory global_trajectory;
    /// @brief global percep parking slots
    HmiGlbPrcpPrkgSlots global_percep_parking_slots;
    /// @brief global perception obstacle
    HmiGlbPrcpObstacle global_perception_obstacle;
    /// @brief distance of already mapping (m)
    float map_distance;
    /// @brief rest distance of allow to do mapping (m)
    float rest_map_distance;
    /// @brief num of speed bumps in map
    std::uint8_t num_speed_bumps;
    std::uint8_t padding[7];
  };
  struct alignas(8) HmiMapDisplay {
    /// @brief Map info, only used when update_map_info is true
    HmiMapInfo map_info;
    std::uint8_t padding[1];
    /// @brief Choose update the map info or map headers
    bool update_map_info;
    /// @brief Num of maps, not used when update_map_info is true
    std::int8_t num_maps;
    /// @brief the target parking slot Id matches new Id
    bool is_match;
    /// @brief matches new Id
    std::int32_t new_id;
    /// @brief All map headers, not used when update_map_info is true
    MapDisplayData::MapCommonHeader header;
  };
  struct alignas(8) HmiFusionObstacles {
    std::int64_t timestamp;
    std::uint8_t padding[4];
    std::uint32_t num_obstacle_list;
    std::array<FusionObstacleData::FusionObstacleOutPut,
               FusionObstacleData::kMaxObstaclesSize>
        obstacle_list;
  };
  struct alignas(8) HmiHpaRender {
    HmiMapDisplay hmi_map_display;
    HmiLocalization hmi_localization;
    HmiFusionObstacles hmi_fusion_obstacles;
    HmiVpaState hmi_vpa_state;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit HmiHpaRenderData(std::int64_t timestamp) noexcept;

  HmiHpaRenderData() noexcept;
  ~HmiHpaRenderData() noexcept final = default;
  HmiHpaRenderData(HmiHpaRenderData const &) = default;
  HmiHpaRenderData &operator=(HmiHpaRenderData const &) = default;
  HmiHpaRenderData(HmiHpaRenderData &&) = default;
  HmiHpaRenderData &operator=(HmiHpaRenderData &&) = default;

  /// @brief Get Hmi Hpa Render
  ///
  /// @return hmi_hpa_render_
  HmiHpaRender const &GetHmiHpaRender() const noexcept;

  /// @brief Mutable Hmi Hpa Render
  ///
  /// @return hmi_hpa_render_
  HmiHpaRender &MutableHmiHpaRender() noexcept;

  /// @brief Set Hmi Hpa Render
  ///
  /// @param hmi_render Hmi Hpa render
  void SetHmiHpaRender(HmiHpaRender const &hmi_hpa_render) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

private:
  HmiHpaRender hmi_hpa_render_;
};

} // namespace autodrive

#endif // AUTODRIVE_INTERFACE_HMI_HPA_ENGINE_DATA_H_
