//
#ifndef AUTODRIVE_MODEL_CAMERA_EXTRINSICS_DATA_H_
#define AUTODRIVE_MODEL_CAMERA_EXTRINSICS_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

namespace autodrive {

// CameraID
enum class CameraID : std::int8_t {
  CAMERA_DIRECTION_UNKNOWN = 0,
  // 环视鱼眼相机
  CAMERA_DIRECTION_BACK = 1,
  CAMERA_DIRECTION_FRONT = 2,
  CAMERA_DIRECTION_LEFT = 3,
  CAMERA_DIRECTION_RIGHT = 4,
  // 前后视相机
  CAMERA_DIRECTION_PIN_REAR = 5,
  CAMERA_DIRECTION_PIN_FRONT = 6,
  // 广角相机
  CAMERA_DIRECTION_OMNI_BACK = 7,
  CAMERA_DIRECTION_OMNI_FRONT = 8,
  CAMERA_DIRECTION_OMNI_LEFT = 9,
  CAMERA_DIRECTION_OMNI_RIGHT = 10
};

// Camera Extrinsics data
class CameraExtrinsicsData final : public CommonMsgData {
 public:
  /// @brief Max Extrinsic Size
  static constexpr std::uint8_t kMaxExtrinsicsSize{5U};

  struct alignas(8) Extrinsic {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    std::uint32_t num_extrinsic;
    CameraID cam_id;
    std::uint8_t padding[3];
  };

  struct alignas(8) Extrinsics {
    std::uint8_t num_extrinsics;
    std::uint8_t padding[7];
    std::array<Extrinsic, kMaxExtrinsicsSize> extrinsics;
  };

  CameraExtrinsicsData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit CameraExtrinsicsData(std::int64_t timestamp) noexcept;
  ~CameraExtrinsicsData() noexcept = default;
  CameraExtrinsicsData(CameraExtrinsicsData const&) = default;
  CameraExtrinsicsData& operator=(CameraExtrinsicsData const&) = default;
  CameraExtrinsicsData(CameraExtrinsicsData&&) = default;
  CameraExtrinsicsData& operator=(CameraExtrinsicsData&&) = default;

  /// @brief Get function mode
  ///
  /// @return function_mode_
  Extrinsics const& GetExtrinsics() const noexcept;

  /// @brief Get mutable function mode
  ///
  /// @return function_mode_
  Extrinsics& MutableExtrinsics() noexcept;

  /// @brief Set function mode
  ///
  /// @return function_mode_
  void SetExtrinsics(const Extrinsics& extrinsics) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  Extrinsics extrinsics_;
};

}  // namespace autodrive

#endif
