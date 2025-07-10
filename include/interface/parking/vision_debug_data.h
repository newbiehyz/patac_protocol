//
#ifndef AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DEBUG_DATA_H_
#define AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DEBUG_DATA_H_

#include <foundation/type/common_msg_data.h>
#include "type/dynamic_obstacle_debug_info.h"

namespace autodrive {

class VisionDebugData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxDynamicObstacleSize{64U};
  struct alignas(8) DynamicObstaclesDebugInfo {
    std::int32_t num_obstacles;
    std::array<DynamicObstacleDebugInfo, kMaxDynamicObstacleSize> obstacles_debug_info;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit VisionDebugData(std::int64_t timestamp) noexcept;

  VisionDebugData() noexcept;
  ~VisionDebugData() noexcept final = default;
  VisionDebugData(VisionDebugData const &) = default;
  VisionDebugData &operator=(VisionDebugData const &) = default;
  VisionDebugData(VisionDebugData &&) = default;
  VisionDebugData &operator=(VisionDebugData &&) = default;

  /// @brief Get Dynamic Obstacle data debug info
  ///
  /// @return dynamic_obstacles_debug_info_
  DynamicObstaclesDebugInfo const &GetDynamicObstaclesDebugInfo() const noexcept;

  /// @brief Mutable Dynamic Obstacle debug info
  ///
  /// @return dynamic_obstacles_debug_info_
  DynamicObstaclesDebugInfo &MutableDynamicObstaclesDebugInfo() noexcept;

  /// @brief Set Dynamic Obstacle debug info
  ///
  /// @param lane_marker_data Lane marker dynamic_obstacle_data
  void SetDynamicObstaclesDebugInfo(
      DynamicObstaclesDebugInfo const &dynamic_obstacle_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
     DynamicObstaclesDebugInfo dynamic_obstacles_debug_info_;
};

}  // namespace autodrive

#endif // AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DEBUG_DATA_H_
