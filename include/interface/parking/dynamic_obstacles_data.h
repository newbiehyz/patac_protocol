//
#ifndef AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DATA_H_
#define AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DATA_H_

#include <foundation/type/common_msg_data.h>
#include "type/dynamic_obstacle.h"

namespace autodrive {

class DynamicObstaclesData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxDynamicObstacleSize{64U};
  struct alignas(8) DynamicObstacles {
    std::int32_t cipv_id;
    std::int32_t num_obstacles;
    std::array<DynamicObstacle, kMaxDynamicObstacleSize> obstacles;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit DynamicObstaclesData(std::int64_t timestamp) noexcept;

  DynamicObstaclesData() noexcept;
  ~DynamicObstaclesData() noexcept final = default;
  DynamicObstaclesData(DynamicObstaclesData const&) = default;
  DynamicObstaclesData& operator=(DynamicObstaclesData const&) = default;
  DynamicObstaclesData(DynamicObstaclesData&&) = default;
  DynamicObstaclesData& operator=(DynamicObstaclesData&&) = default;

  /// @brief Get Dynamic Obstacle data
  ///
  /// @return dynamic_obstacles_
  DynamicObstacles const& GetDynamicObstacles() const noexcept;

  /// @brief Mutable Dynamic Obstacle data
  ///
  /// @return dynamic_obstacles_
  DynamicObstacles& MutableDynamicObstacles() noexcept;

  /// @brief Set Dynamic Obstacle data
  ///
  /// @param lane_marker_data Lane marker dynamic_obstacle_data
  void SetDynamicObstacles(
      DynamicObstacles const& dynamic_obstacle_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  DynamicObstacles dynamic_obstacles_;
};

}  // namespace autodrive

#endif
