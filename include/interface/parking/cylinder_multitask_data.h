//
#ifndef AUTODRIVE_INTERFACE_CYLINDER_MULTITASK_DATA_H_
#define AUTODRIVE_INTERFACE_CYLINDER_MULTITASK_DATA_H_

#include <array>
#include <cstdint>

#include <foundation/type/common_msg_data.h>
#include "type/obstacle_raw.h"
#include "type/parsing_base.h"
#include "type/perception_base.h"
#include "type/real3d.h"

namespace autodrive {

class CylinderMultiTaskData final : public CommonMsgData {
 public:
  // constexpr and macro
  static constexpr std::uint32_t kMaxCameraNumber{4U};

  struct alignas(8) CameraValidFlag{
    ImageId sensor_id;
    bool obstacle_raw_data_valid;
    bool real3d_data_valid;
    bool parsing_data_valid;
    std::uint8_t padding[4];
  };

  struct alignas(8) CylinderMultiTask {
    std::uint8_t num_camera_flag_data;
    std::uint8_t padding[7];
    std::array<CameraValidFlag, kMaxCameraNumber> camera_flag_data;
    std::array<ObstacleRawsData, kMaxCameraNumber> obstacle_raw_data;
    std::array<Real3DData, kMaxCameraNumber> real3d_data;
  };

  explicit CylinderMultiTaskData(std::int64_t timestamp) noexcept;
  CylinderMultiTaskData() noexcept;
  ~CylinderMultiTaskData() noexcept final = default;
  CylinderMultiTaskData(CylinderMultiTaskData const&) = default;
  CylinderMultiTaskData& operator=(CylinderMultiTaskData const&) = default;
  CylinderMultiTaskData(CylinderMultiTaskData&&) = default;
  CylinderMultiTaskData& operator=(CylinderMultiTaskData&&) = default;

  /// @brief Get cylinder multi task
  ///
  /// @return cylinder_multitask_data_
  CylinderMultiTask const& GetCylinderMultiTask() const noexcept;

  /// @brief Mutable cylinder multi task data
  ///
  /// @return cylinder_multitask_data_
  CylinderMultiTask& MutableCylinderMultiTask() noexcept;

  /// @brief Set cylinder multitask data
  ///
  /// @param cylinder_multitask_data CylinderMultiTask
  void SetCylinderMultiTask(
      CylinderMultiTask const& cylinder_multitask_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  CylinderMultiTask cylinder_multitask_data_;
};

}  // namespace autodrive

#endif // AUTODRIVE_INTERFACE_CYLINDER_MULTITASK_DATA_H_