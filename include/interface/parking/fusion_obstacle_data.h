//
#ifndef AUTODRIVE_INTERFACE_FUSION_OBSTACLE_DATA_H_
#define AUTODRIVE_INTERFACE_FUSION_OBSTACLE_DATA_H_

#include <foundation/type/common_msg_data.h>
#include <type/perception_base.h>
#include <array>

namespace autodrive {

class FusionObstacleData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxObstaclesSize{52U};

  enum class FusionObstacleType : std::uint8_t {
    /// @brief Dynamic obstacle
    /// @brief FusionOutput
    kVehicle = 0,
    /// @brief FusionOutput
    kPedestrain = 1,
    /// @brief FusionOutput
    kCyclist = 2,
    /// @brief FusionOutput
    kAnimal = 3,
    /// @brief Static obstacle
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

  enum class VehicleSubType : std::uint8_t {
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

  enum class FusionObstacleStaticSubType : std::uint8_t {
    kParkLockTypeUnknown = 0,
    kParkLockTypeOpen = 1,
    kParkLockTypeClose = 2,
    kFireHydrantTypeAir = 3,
    kFireHydrantTypeGnd = 4
  };

  enum class ObstacleMotionCategory : std::uint8_t {
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

  enum class ObstacleMotionOrientation : std::uint8_t {
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

  enum class ObstacleMotionStatus : std::uint8_t {
    kUnknown = 0,
    kMoving = 1,
    kOncoming = 2,
    kStopped = 3,
    kStationary = 4
  };

  struct alignas(8) TwoPoints {
    // @brief Uss_only using ,default value will set
    Vector3f pos1;
    Vector3f pos2;
  };

  struct alignas(8) FusionObstacleOutPut {
    /// @brief Continuous tracking frames
    std::uint32_t age;
    /// @brief is_updated
    bool is_updated;
    /// @brief Bit1:IPM; Bit2:USS; Bit3:Mono;
    /// For example,IPM only: uint8_t(1);IPM+USS: uint8_t(3);
    std::uint8_t sensor_source;
    std::uint8_t padding_1[2];
    /// @brief Fusion id
    std::uint32_t id;
    /// @brief Vision id
    std::uint32_t vision_id;
    /// @brief Uss id
    std::uint32_t uss_id;
    /// @brief Mono id
    std::uint32_t mono_id;
    /// @brief Position [x, y] center point only for fusion and vision
    Vector3f pos;
    /// @brief Position variance only for fusion and vision
    Vector3f pos_std;
    /// @brief Velocity [x, y], uss_only will give default value 0
    Vector3f vel;
    /// @brief Velocity  wcs[x, y], uss_only will give default value 0
    Vector3f vel_abs_world;
    /// @brief Acceleration [x, y],uss_only will give default value 0
    Vector3f acc;
    /// @brief Acceleration wcs [x, y],uss_only will give default value 0
    Vector3f acc_abs_world;
    /// @brief Heading for this object
    float heading;
    /// @brief Momentary angular speed [rad/s] for this object
    float angular_speed;
    /// @brief Obstacle Probability
    float obstacle_conf;
    /// @brief [m] Only for fusion and vision
    float width;
    /// @brief [m] Only for fusion and vision
    float length;
    /// @brief [m] Only for fusion and vision
    float height;
    /// @brief [s] ttc for this object
    float ttc;
    /// @brief Type
    FusionObstacleType type;
    /// @brief Vehicle subtype
    VehicleSubType veh_sub_type;
    // static obs subtype
    FusionObstacleStaticSubType static_sub_type;
    /// @brief Motion stats, uss_only set default value unkown
    ObstacleMotionStatus motion_state;
    /// @brief Two point uss_only usinng
    TwoPoints uss_pos;
    /// @brief Motion category, uss_only set default value invalid
    ObstacleMotionCategory motion_category;
    /// @brief Motion orientation, uss_only set default value undefined
    ObstacleMotionOrientation motion_orientation;
    std::uint8_t padding_2[6];
  };

  struct alignas(8) FusionObstacles {
    std::int64_t timestamp;
    std::uint8_t padding[4];
    std::uint32_t num_obstacle_list;
    std::array<FusionObstacleOutPut, kMaxObstaclesSize> obstacle_list;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit FusionObstacleData(std::int64_t timestamp) noexcept;

  FusionObstacleData() noexcept;
  ~FusionObstacleData() noexcept final = default;
  FusionObstacleData(FusionObstacleData const&) = default;
  FusionObstacleData& operator=(FusionObstacleData const&) = default;
  FusionObstacleData(FusionObstacleData&&) = default;
  FusionObstacleData& operator=(FusionObstacleData&&) = default;

  /// @brief Get Fusion obstacle data
  ///
  /// @return fusion_obstacle_data_
  FusionObstacles const& GetFusionObstacle() const noexcept;

  /// @brief Mutable Fusion obstacle data
  ///
  /// @return fusion_obstacle_data_
  FusionObstacles& MutableFusionObstacle() noexcept;

  /// @brief Set Fusion obstacle data
  ///
  /// @param fusion_obstacle_data_ fusion obstacle data
  void SetFusionObstacle(FusionObstacles const& fusion_obstacle_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  FusionObstacles fusion_obstacle_data_;
};

}

#endif