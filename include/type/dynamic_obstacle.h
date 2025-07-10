//
#ifndef AUTODRIVE_TYPE_DYNAMIC_OBSTACLE_H_
#define AUTODRIVE_TYPE_DYNAMIC_OBSTACLE_H_

#include "perception_base.h"

namespace autodrive {

static constexpr std::uint32_t kMaxSigmaSize{4U};
static constexpr std::uint32_t kMaxDynamicObstaclePropertySize{4U};
static constexpr std::uint32_t kMaxPropertyNameSize{4U};
static constexpr std::uint32_t kMaxPropertyNameNumsSize{20U};
static constexpr std::uint32_t kMaxPropertyTypeSize{4U};
static constexpr std::uint32_t kMaxPropertyConfSize{4U};
static constexpr std::uint32_t kMaxCategorySize{8U};

enum class ObstacleType : std::uint8_t {
  kVehicleRear = 0,
  kVehicleFull = 1,
  kPedestrian = 2,
  kCyclist = 18,
  kAnimal = 28,
  kUnknown = 50
};

enum class DynamicVehicleType : std::uint8_t {
  kUnknown = 0,
  kCar = 1,
  kSUV = 2,
  kVan = 3,
  kBus = 4,
  kTruck = 5,
  kAmbulance = 6,
  kPoliceCar = 7,
  kFireTruck = 8,
  kTricycle = 9,
  kSpecialCar = 10,
  kTinyCar = 11,
  kMotor = 12
};

enum class MotionStatus : std::uint8_t {
  kInvalid = 0,
  kUnknown = 1,
  kMoving = 2,
  kStationary = 3,
  kStopped = 4,
  kMovingSlowly = 5,
  kEnumSize = 6
};

enum class MotionCategory : std::uint8_t {
  kInvalid = 0,
  kUndefined = 1,
  kPassing = 2,
  kPassingIn = 3,
  kPassingOut = 4,
  kCloseCutIn = 5,
  kMovingIn = 6,
  kMovingOut = 7,
  kCrossing = 8,
  kLeftTurnAP = 9,
  kRightTurnAP = 10,
  kMoving = 11,
  kPreceding = 12,
  kOncoming = 13,
  kEnumSize = 14
};

enum class MotionOrientation : std::uint8_t {
  kUndefined = 0,
  kDriftingRight = 1,
  kCrossingRight = 2,
  kOncomingDriftRight = 3,
  kOncoming = 4,
  kOncomingDriftLeft = 5,
  kCrossingLeft = 6,
  kDriftingLeft = 7,
  kPreceding = 8
};

enum class VisibleSide : std::uint8_t { kUnknown = 0, kRight = 1, kLeft = 2 };

enum class VisibleRect : std::uint8_t { kUnknown = 0, kFront = 1, kBack = 2 };

enum class LaneAssignment : std::int8_t {
  kLeftNextNext = -2,
  kLeftNext = -1,
  kEgoLane = 0,
  kRightNext = 1,
  kRightNextNext = 2,
  kLaneUnknown = 3
};

struct alignas(8) DynamicObstVelocity {
  float vx;
  float vy;
  float vz;
  std::uint8_t padding[4];
};

struct alignas(8) DynamicObstAccelerate {
  float ax;
  float ay;
  float az;
  std::uint8_t padding[4];
};

/// @brief The four corner points of the target enveloping frame under BEV &&
/// The midpoint of the four sides && The center point
enum class OutputObsPositionType : std::uint8_t {
  kUnknownPoint = 0,
  kRearLeftPoint = 1,
  kRearCenterPoint = 2,
  kRearRightPoint = 3,
  kFrontLeftPoint = 4,
  kFrontCenterPoint = 5,
  kFrontRightPoint = 6,
  kLeftCenterPoint = 7,
  kRightCenterPoint = 8,
  kCenterPoint = 9
};

struct alignas(8) ObstacleAngle {
  /// @brief Left angle variance
  float angle_left_v;
  /// @brief Left angle value
  float angle_left;
  /// @brief Middle angle variance
  float angle_middle_v;
  /// @brief Middle angle value
  float angle_middle;
  /// @brief Right angle variance
  float angle_right_v;
  /// @brief Right angle value
  float angle_right;
  /// @brief Side angle variance
  float angle_side_v;
  /// @brief Side angle value
  float angle_side;
};

/// @brief Struct representing world space information
struct alignas(8) DynamicObstWorldSpaceInfo {
  /// @brief Yaw
  float yaw;
  /// @brief Length
  float length;
  /// @brief Width
  float width;
  /// @brief Height
  float height;
  /// @brief 3D bbox points
  Polygon poly;
  /// @brief Polygon points on the ground in vehicle coordinate system
  Polygon poly_vcs;
  /// @brief Position in vehicle coordinate system
  /// Position, VCS coordinate system, with the rear axle center as the origin
  /// (longitudinal distance, lateral distance, 0).
  Vector3f position;
  /// RBC Position, VCS coordinate system, with the rear axle center as the
  /// origin (longitudinal distance, lateral distance, 0).
  Vector3f rbc_position;
  /// @brief Collision time ttc_const_vel (NoOutput)
  float ttc;
  std::int32_t traversable;
  DynamicObstVelocity vel;
  /// @brief Absolute velocity
  /// (longitudinal absolute velocity, lateral absolute velocity, 0)
  DynamicObstVelocity vel_abs_world;
  /// @brief Absolute acceleration
  /// (longitudinal absolute velocity, lateral absolute Velocity, 0)
  DynamicObstAccelerate acc_abs_world;
  ///@brief Relative acceleration
  ///(longitudinal relative velocity, lateral relative Veloocity, 0)
  DynamicObstAccelerate acc;
  /// @brief Motion status
  MotionStatus motion_status;
  /// @brief Motion category
  MotionCategory motion_category;
  /// @brief Position observation point OutputObsPositionType =>
  /// centroid_mode (0 ~ 9)
  OutputObsPositionType position_type;
  /// @brief Visual edge
  VisibleSide visible_side;
  /// @brief Yaw angular speed
  float yaw_rate;

  /// @brief Absolute velocity variance [absoluteLongVelocityStd, 0, 0,
  /// absoluteLatVelocityStd]
  std::array<float, kMaxSigmaSize> sigma_vel;

  /// @brief Heading angle variance
  float sigma_yaw;
  /// @brief Width variance
  float sigma_width;
  /// @brief Height variance
  float sigma_height;
  /// @brief Length variance
  float sigma_length;
  /// @brief Position variance [longDistanceStd, 0, 0, latDistanceStd]
  std::array<float, kMaxSigmaSize> sigma_position;
  /// @brief RBC Position variance [longDistanceStd, 0, 0, latDistanceStd]
  std::array<float, kMaxSigmaSize> sigma_rbc_position;

  /// @brief Angular velocity variance angularSpeedStd
  float sigma_yaw_rate;
  /// @brief Visible front or rear visible_rect (NoOutput)
  std::int32_t head_rear_type;
  /// @brief Absolute acceleration variance [absoluteLongAccStd, 0, 0,
  /// absoluteLatAccStd] (fixed value temporarily)
  std::array<float, kMaxSigmaSize> sigma_acc_ref;

  /// @brief Azimuth (NoOutput) [angleRight, angleRightStd, angleLeft,
  /// angleLeftStd, angleSide, angleSideStd, angleMid, angleMidStd]
  ObstacleAngle obj_angle;

  /// @brief Visual rect
  VisibleRect visible_rect;
  /// @brief Motion orientation
  MotionOrientation motion_orientation;
  LaneAssignment lane_assignment;
  std::int8_t cipv;
  std::uint8_t padding[4];
};

/// @brief Struct representing an obstacle detected by the perception system
struct alignas(8) DynamicObstacle {
  /// @brief Ms, ReservedFields
  std::int32_t life_time;
  /// @brief Camera index
  std::int32_t cam_id;
  /// @brief Obstacle timestamp
  std::int64_t time_stamp;
  /// @brief Confidence
  float conf;
  /// @brief Continuous tracking frames
  std::uint32_t age;
  /// @brief Information about the image space location of the obstacle
  ImageSpace img_info;
  /// @brief Information about the world space location of the obstacle
  DynamicObstWorldSpaceInfo world_info;

  /// @brief ReservedFields
  std::int32_t valid_info;
  /// @brief ReservedFields
  std::int32_t select_level;
  std::uint16_t id;
  /// @brief Type of the obstacle
  ObstacleType type;
  DynamicVehicleType sub_type;
  std::uint32_t num_categories;
  /// @brief ReservedFields
  std::array<Category, kMaxCategorySize> categories;
};

}  // namespace autodrive

#endif