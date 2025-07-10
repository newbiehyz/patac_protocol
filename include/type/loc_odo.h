//
#ifndef AUTODRIVE_TYPE_LOC_ODO_H_
#define AUTODRIVE_TYPE_LOC_ODO_H_

#include <array>
#include <cstdint>

#include <type/basic.h>

namespace autodrive {

static constexpr std::uint32_t kMaxPoseCovSize{32U};
static constexpr std::uint32_t kMaxTwistCovSize{32U};

struct alignas(8) Quaternion4d {
  double x;
  double y;
  double z;
  double w;
};

struct alignas(8) Odometry {
  /// @brief Time stamp
  std::uint64_t stamp;
  /// @brief Quaternion xyzw
  Quaternion4d quaternion;
  /// @brief Position
  Vector3d position;
  /// @brief Sequence id
  std::uint32_t seq_id;
  /// @brief Num of pose covariance
  std::uint32_t num_pose_cov;
  /// @brief Pose cov
  std::array<double, kMaxPoseCovSize> pose_cov;
  /// @brief Linear velocity
  Vector3d linear_velocity;
  /// @brief Angular velocity
  Vector3d angular_velocity;
  std::uint8_t padding[4];
  /// @brief Number of twist covariance
  std::uint32_t num_twist_cov;
  /// @brief Twist cov
  std::array<double, kMaxTwistCovSize> twist_cov;
  /// @brief Offset of map
  Vector3d map_offset;
  /// @brief Velocity
  Vector3d velocity;
  /// @brief Acceleration
  Vector3d acceleration;
  /// @brief Floor
  float floor;
  std::uint8_t padding_2[4];
};

}  // namespace autodrive

#endif
