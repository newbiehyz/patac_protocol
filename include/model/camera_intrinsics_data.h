//
#ifndef AUTODRIVE_MODEL_CAMERA_INTRINSICS_DATA_H_
#define AUTODRIVE_MODEL_CAMERA_INTRINSICS_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "model/camera_extrinsics_data.h"

namespace autodrive {

// Camera Intrinsics data
class CameraIntrinsicsData final : public CommonMsgData {
 public:
  /// @brief Max Intrinsics size
  static constexpr std::uint8_t kMaxCameraSize{5U};
  /// @brief max number of k
  static constexpr std::uint8_t kMaxkSize{14U};
  /// @brief max number of p
  static constexpr std::uint8_t kMaxpSize{6U};
  /// @brief max number of s
  static constexpr std::uint8_t kMaxsSize{2U};

  struct alignas(8) Intrinsic {
    float cx;
    float cy;
    float cod_x;
    float cod_y;
    float fx;
    float fy;
    CameraID cam_id;
    std::uint8_t padding[3];
    std::uint32_t num_k;
    std::uint32_t num_p;
    std::uint32_t num_s;
    std::array<float, kMaxkSize> k;
    std::array<float, kMaxpSize> p;
    std::array<float, kMaxsSize> s;
  };

  struct alignas(8) Intrinsics {
    std::uint8_t num_intrinsics;
    std::uint8_t padding[7];
    std::array<Intrinsic, kMaxCameraSize> intrinsics;
  };

  CameraIntrinsicsData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit CameraIntrinsicsData(std::int64_t timestamp) noexcept;
  ~CameraIntrinsicsData() noexcept = default;
  CameraIntrinsicsData(CameraIntrinsicsData const&) = default;
  CameraIntrinsicsData& operator=(CameraIntrinsicsData const&) = default;
  CameraIntrinsicsData(CameraIntrinsicsData&&) = default;
  CameraIntrinsicsData& operator=(CameraIntrinsicsData&&) = default;

  /// @brief Get function mode
  ///
  /// @return function_mode_
  Intrinsics const& GetIntrinsics() const noexcept;

  /// @brief Get mutable function mode
  ///
  /// @return function_mode_
  Intrinsics& MutableIntrinsics() noexcept;

  /// @brief Set function mode
  ///
  /// @return function_mode_
  void SetIntrinsics(const Intrinsics& intrinsics) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  Intrinsics intrinsics_;
};

}  // namespace autodrive

#endif