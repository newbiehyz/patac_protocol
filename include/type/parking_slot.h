//
#ifndef AUTODRIVE_TYPE_PARKING_SLOT_H_
#define AUTODRIVE_TYPE_PARKING_SLOT_H_

#include <array>

#include <type/basic.h>

namespace autodrive {

static constexpr std::uint32_t kMaxCornerSize{4U};
static constexpr std::uint32_t kMaxEndPointsSize{2U};
static constexpr std::uint32_t kMaxPolygonPointsSize{4U};
static constexpr std::uint32_t kMaxStrIdSize{128U};
static constexpr std::uint32_t kMaxBoundaryPointsSize{3U};

struct alignas(8) CornerInfo {
  Vector3f point_vcs;
  Vector3f point_world;
  Vector3f point_cov;
  bool valid;
  std::uint8_t padding[7];
};

struct alignas(8) PoseInfo {
  Vector3f position_vcs;
  Vector3f position_world;
  float length;
  float width;
  float slotline_w0;
  float slotline_w1;
  float yaw_vcs;
  float yaw_world;
  float alpha_vcs;
  float alpha_world;
  std::uint32_t num_corners;
  std::uint8_t padding[4];
  std::array<Vector3f, kMaxCornerSize> corners_vcs;
  std::array<Vector3f, kMaxCornerSize> corners_world;
};

struct alignas(8) ParkingRodInfo {
  std::array<Vector3f, kMaxEndPointsSize> end_points_vcs;
  std::array<Vector3f, kMaxEndPointsSize> end_points_world;
  std::uint32_t num_end_points;
  bool valid;
  std::uint8_t padding[3];
};

enum class ParkingType : std::uint8_t {
  kUnknown = 0,
  kVerticalParking = 1,
  kParallelParking = 2,
  kObliqueParking = 3
};

enum class OccupancyStatus : std::uint8_t {
  kUnknown = 0,
  kOccupied = 1,
  kNotOccupied = 2
};

enum class ParkingSource : std::uint8_t {
  kUnspecified = 0,
  kIpm = 1,
  kUss = 2,
  kUserSel = 3
};

enum class ParkingStage : std::uint8_t {
  kUninit = 0,
  kUnconverged = 1,
  kConverged = 2,
  kUpdateError = 3,
  kLost = 4,
  kFixed = 5
};

enum class ParkingLockStatus : std::uint8_t {
  kUnknown = 0,
  kOpen = 1,
  kClose = 2
};

struct alignas(8) ParkingLockInfo {
  /// @brief Position in vcs coordinate
  Vector3f point_vcs;
  /// @brief Position in vcs coordinate
  Vector3f point_world;
  /// @brief Status of parking lock
  ParkingLockStatus parking_status;
  /// @brief Validity of parking lock
  bool valid;
  std::uint8_t padding[6];
};

enum class ParkingReleaseMode : std::uint8_t {
  kNoneMode = 0,
  kRealTime = 1,
  kTracking = 2,
};

enum class BoundaryStatus : std::uint8_t {
  kUnspecified = 0,
  kLow = 1,
  kHigh = 2,
  kMid = 3,
};

enum class BoundaryFixStatus : std::uint8_t {
  kUnspecified = 0,
  kUnfixed = 1,
  kFixed = 2,
};

struct alignas(8) BoundaryInfo {
  std::array<Vector3f, kMaxBoundaryPointsSize> points_vcs;
  std::array<Vector3f, kMaxBoundaryPointsSize> points_world;
  std::array<BoundaryStatus, kMaxBoundaryPointsSize> boundaries_status;
  std::uint8_t padding_1[1];
  std::array<float, kMaxBoundaryPointsSize> boundaries_height;
  std::uint32_t num_boundary_points;
  std::array<BoundaryFixStatus, kMaxBoundaryPointsSize> boundaries_fix_status;
  std::uint8_t padding_2[1];
  /// @brief The distance between the parking space boundary and the wireframe
  std::array<Vector3f, kMaxBoundaryPointsSize> points_delta_ss;
};

enum class OccupyObjType : std::uint8_t {
  /// @brief Dynamic obstacle,the type in purple color can be output currently.
  /// @brief FusionOutput
  kVehicle = 0,
  /// @brief FusionOutput
  kPedestrain = 1,
  /// @brief FusionOutput
  kCyclist = 2,
  kAnimal = 3,

  /// @brief Static obstacle,the type in purple color can be output currently.
  /// @brief FusionOutput
  kTrafficCone = 50,
  /// @brief FusionOutput
  kCementColumn = 51,
  /// @brief FusionOutput
  kParkingLock = 52,
  /// @brief FusionOutput
  kAFrameSign = 53,
  kCurb = 54,
  kFence = 55,
  kWaterBarrier = 56,
  kTrolley = 57,
  kWall = 58,

  kUnknown = 100
};

struct alignas(8) ParkingSlotInfo {
  /// @brief Parking slot id
  std::int32_t id;
  /// @brief Parking valid information
  std::int32_t valid;
  /// @brief Parking confidence
  float conf;
  std::uint32_t num_polygon_points;
  /// @brief Parking corner tracking results
  std::array<CornerInfo, kMaxPolygonPointsSize> polygon_points;
  /// @brief Structured parking information extracted from polygon_points
  PoseInfo pose;
  /// @brief Parking rod position
  ParkingRodInfo parking_rod;
  /// @brief Parking lock information
  ParkingLockInfo parking_lock_info;
  /// @brief Parking boundary, correspond to poly 12,23,30
  BoundaryInfo boundary;

  std::uint16_t num_str_id;
  /// @brief Parking type
  ParkingType type;
  /// @brief Parking occupancy status
  OccupancyStatus occupancy_status;
  /// @brief Parking source
  ParkingSource parking_source;
  /// @brief Parking stage
  ParkingStage stage;
  /// @brief Parking release mode
  ParkingReleaseMode parking_release_mode;
  OccupyObjType occupancy_obs_type;
  /// @brief Global id, UUID
  std::array<char, kMaxStrIdSize> str_id;
};

}  // namespace autodrive

#endif