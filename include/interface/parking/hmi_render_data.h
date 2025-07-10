//
#ifndef AUTODRIVE_INTERFACE_HMI_ENGINE_DATA_H_
#define AUTODRIVE_INTERFACE_HMI_ENGINE_DATA_H_

#include <cstdint>

#include "fusion_obstacle_data.h"
#include "fusion_parking_slots_data.h"

namespace autodrive
{

class HmiRenderData final : public CommonMsgData
{
public:
  struct alignas(8) HmiRender
  {
    FusionObstacleData::FusionObstacles obstacles;
    FusionParkingSlotsData::FusionParkingSlots parkingSlots;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit HmiRenderData(std::int64_t timestamp) noexcept;

  HmiRenderData() noexcept;
  ~HmiRenderData() noexcept final = default;
  HmiRenderData(HmiRenderData const&) = default;
  HmiRenderData& operator=(HmiRenderData const&) = default;
  HmiRenderData(HmiRenderData&&) = default;
  HmiRenderData& operator=(HmiRenderData&&) = default;

  /// @brief Get Hmi Render
  ///
  /// @return hmi_render_
  HmiRender const& GetHmiRender() const noexcept;

  /// @brief Mutable Hmi Render
  ///
  /// @return hmi_render_
  HmiRender& MutableHmiRender() noexcept;

  /// @brief Set Hmi Render
  ///
  /// @param hmi_render Hmi render
  void SetHmiRender(HmiRender const& hmi_render) noexcept;

/// @brief Clear data
void Clear() noexcept;

private:
  HmiRender hmi_render_;
};

} // namespace autodrive

#endif // AUTODRIVE_INTERFACE_HMI_ENGINE_DATA_H_