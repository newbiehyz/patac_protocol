//
#ifndef AUTODRIVE_INTERFACE_STATIC_OBSTACLES_DATA_H_
#define AUTODRIVE_INTERFACE_STATIC_OBSTACLES_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/basic.h"
#include "type/perception_base.h"

namespace autodrive {

class StaticObstaclesData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxStrIdSize{20U};
  static constexpr std::uint32_t kMaxStaticObstacleSize{50U};
  static constexpr std::uint32_t kMaxStaticObstacleAttrValueSize{10U};

  enum class StaticObstacleType : std::uint8_t {
    kUnknown = 0,
    /// @brief Traffic cone
    kTrafficCone = 1,
    /// @brief Park lock
    kParkLock = 2,
    /// @brief Traffic sign, reserved type
    kTrafficSign = 3,
    /// @brief Traffic light, reserved type
    kTrafficLight = 4,
    /// @brief Vertical pole, reserved type
    kVerticalPole = 5,
    /// @brief Park column
    kParkColumn = 6,
    /// @brief A frame sign
    kParkAFrameSign = 7,
    /// @brief Box, reserved type
    kParkBox = 8,
    /// @brief Barrel, reserved type
    kParkBarrel = 9,
    /// @brief Water barrier
    kWaterBarrier = 10,
    /// @brief Traffic bollard
    kTrafficBollard = 11,
    /// @brief Trolley
    kTrolley = 12,
    /// @brief Fire hydrant
    kFireHydrant = 13,
    /// @brief Cement sphere
    kCementSphere = 14,
    /// @brief Charging pile
    kChargingPile = 15,
    /// @brief Reflector
    kReflector = 16,
    /// @brief Distribution box
    kDistributionBox = 17,
    /// @brief Fence
    kFence = 18
  };

  /// @brief Static obstacle orientation
  enum class Orientation : std::uint8_t {
    kUnknown = 0,
    /// @brief Back
    kBack = 1,
    /// @brief Side
    kSide = 2,
    /// @brief Front
    kFront = 3
  };

  /// @brief Static obstacle stage
  enum class ObjectStage : std::uint8_t {
    /// @brief Uninit
    kUninit = 0,
    /// @brief Unconverged
    kUnconverged = 1,
    /// @brief Converged
    kConverged = 2,
    /// @brief Update error
    kUpdateError = 3,
    /// @brief Lost
    kLost = 4,
    /// @brief Fixed
    kFixed = 5
  };

  struct alignas(8) StaticObstacleAttr {
    /// @brief StaticObstacleShape
    std::int32_t shape;
    /// @brief Unit: s
    float cycle_time;
    std::int32_t traversable;
    /// @brief Unit: m
    float height;
    /// @brief Cylinder, uint: m
    float radius;
    /// @brief Object's status
    std::int32_t status;
    /// @brief E.g. speed_limit value = "60"
    char value[kMaxStaticObstacleAttrValueSize];
    std::uint8_t num_value;
    /// @brief Changeable
    bool dynamic;
    std::uint8_t padding[4];
    /// @brief Static obstacle info in img
    ImageSpace img_info;
  };

  enum class StaticObstacleSubType : std::uint8_t {
    /// @brief Uncertain park lock status
    kParkLockTypeUnknown = 0,
    /// @brief Open park lock
    kParkLockTypeOpen = 1,
    /// @brief Close park lock
    kParkLockTypeClose = 2,
    /// @brief Fire hydrant on the wall or park column
    kFireHydrantTypeAir = 3,
    /// @brief Fire hydrant on the ground
    kFireHydrantTypeGnd = 4
  };

  /// @brief Static obstacle shape, only rectangle shape is used, others are
  /// reserved shapes
  enum class StaticObstacleShape : std::uint8_t {
    /// @brief Uncertain shape
    kUnknown = 0,
    /// @brief Rectangle
    kRectangle = 1,
    /// @brief Triangle
    kTriangle = 2,
    /// @brief Round
    kRound = 3,
    /// @brief Cylinder
    kCylinder = 4
  };

  /// @brief Struct representing one static obstacle detected by the perception
  /// system
  struct alignas(8) StaticObstacle {
    std::int32_t id;
    /// @brief Camera index
    std::int32_t cam_id;
    /// @brief Obstacle timestamp
    std::int64_t time_stamp;
    /// @brief Confidence, 0~100
    std::int32_t conf;
    /// @brief Static obstacle type
    StaticObstacleType type;
    /// @brief Static obstacle subtype
    StaticObstacleSubType sub_type;
    std::uint8_t padding_1[2];
    /// @brief Represent ground coordinate edge points
    Polygon border;
    /// @brief Unit: ms
    std::int32_t life_time;
    /// @brief Frame count
    std::int32_t age;
    std::array<char, kMaxStrIdSize> str_id;
    std::uint8_t num_str_id;
    std::uint8_t padding_2[3];
    /// @brief Static obstacle attributes
    StaticObstacleAttr attr;
    /// @brief StaticObstacle center position in vcs
    Vector3f position;
  };

  struct alignas(8) StaticObstacles {
    /// @brief Confidence value scale factor
    float conf_scale;
    /// @brief The number of static obstacles
    std::int32_t num_static_obstacles;
    std::array<StaticObstacle, kMaxStaticObstacleSize> static_obstacles;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit StaticObstaclesData(std::int64_t timestamp) noexcept;

  StaticObstaclesData() noexcept;
  ~StaticObstaclesData() noexcept final = default;
  StaticObstaclesData(StaticObstaclesData const&) = default;
  StaticObstaclesData& operator=(StaticObstaclesData const&) = default;
  StaticObstaclesData(StaticObstaclesData&&) = default;
  StaticObstaclesData& operator=(StaticObstaclesData&&) = default;

  /// @brief Get Static Obstacles data
  ///
  /// @return static_obstacles_data_
  StaticObstacles const& GetStaticObstacles() const noexcept;

  /// @brief Mutable Static Obstacles data
  ///
  /// @return static_obstacles_data_
  StaticObstacles& MutableStaticObstacles() noexcept;

  /// @brief Set Static Obstacles data
  ///
  /// @param static_obstacles_data static obstacles data
  void SetStaticObstacles(StaticObstacles const& static_obstacles_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  StaticObstacles static_obstacles_data_;
};

}  // namespace autodrive

#endif
