//
#ifndef AUTODRIVE_TYPE_OBSTACKLLE_H_
#define AUTODRIVE_TYPE_OBSTACKLLE_H_

#include <type/basic.h>
#include <type/perception_base.h>

namespace autodrive {

// constexpr and macro
static constexpr std::uint32_t kMaxTrackingFeatureRawSize{16U};
static constexpr std::uint32_t kMaxKpsClsRaw{64};
static constexpr std::uint32_t kMaxKpsClsRawName{16};
static constexpr std::uint32_t kMaxKeyPointsRaw{16};
static constexpr std::uint32_t kMaxObstacleRaws{64U};
static constexpr std::uint32_t kMaxObstacleRawCategorySize{4U};
static constexpr std::uint32_t kMaxKeyPoints{64U};
static constexpr std::uint32_t kObstacleRawKeyPointsRawSize{4U};
static constexpr std::uint32_t kKeyPointsSize{2U};
static constexpr std::uint32_t kMaxPropertyBboxSize{2U};
static constexpr std::uint32_t kObstacleRawKpsClsRawSize{4U};
static constexpr std::uint32_t kStageOneKeyPointsSize{3U};

enum class VehPropertyPerceptionType : std::uint8_t {
  kVehPerceptionTypeLicensePlate = 0,
  kVehPerceptionTypeWheel = 1,
  kVehPerceptionTypeCnt = 2
};

struct alignas(8) PropertyPerceptionBBox {
  std::uint8_t num_category;
  std::uint8_t padding[7];
  Rect rect;
  float conf;
  int32_t type;
  std::array<Category, kMaxObstacleRawCategorySize> category;
};

struct alignas(8) KeyPoint {
  Vector3f pt;
  float type;
  float conf;
  float boundary_conf_x;
  float boundary_conf_y;
};

struct alignas(8) TrackingFeatureRaw {
  int32_t model_id;
  std::uint16_t num_data;
  std::uint8_t padding[4];
  std::array<std::int8_t, kMaxTrackingFeatureRawSize> data;
};

struct alignas(8) PoseNetRaw {
  float roll_raw;
  float roll_filter;
  float pitch_raw;
  float pitch_filter;
  float yaw_raw;
  float yaw_filter;
  int32_t type;
  float conf;
};

enum class ObstacleRawModelType : std::uint8_t {
  kObstacleRawModelCar = 0,
  kObstacleRawModelFullCar = 1,
  kObstacleRawModelPed = 2,
  kObstacleRawModelHead = 3,
  kObstacleRawModelRare = 4,
  kObstacleRawModelTrafficSign = 5,
  kObstacleRawModelTrafficLight = 6,
  kObstacleRawModelLane = 7,
  kObstacleRawModelRoadSign = 10,
  kObstacleRawModelTrafficArrow = 11,
  kObstacleRawModelYieldMark = 12,
  kObstacleRawModelTrafficCone = 13,
  kObstacleRawModelBarrel = 14,
  kObstacleRawModelSpeedMark = 15,
  kObstacleRawModelCyclist = 16,
  kObstacleRawModelParkingLock = 17,
  kObstacleRawModelSpeedCamera = 18,
  kObstacleRawModelTrafficLightLens = 19,
  kObstacleRawModelParkingSlot = 22,
  kObstacleRawModelFace = 34,
  kObstacleRawModelVehiclePlat = 35,
  kObstacleRawModelLaneCrossPoints = 37,
  kObstacleRawModelVerticalPolePoints = 38,
  kObstacleRawModelTollgate = 39,
  kObstacleRawModelParkingLockClose = 40,
  kObstacleRawModelParkingLockOpen = 41,
  kObstacleRawModelParkingColumn = 42,
  kObstacleRawModelParkingAFrameSign = 43,
  kObstacleRawModelTrafficBollard = 44,
  kObstacleRawModelParkingObstacles = 45,
  kObstacleRawModelHpp = 46,
  kObstacleRawModelFireHydrantAir = 47,
  kObstacleRawModelFireHydrantGnd = 48,
  kObstacleRawModelCementSphere = 49,
  kObstacleRawModelWaterBarrier = 50,
  kObstacleRawModelChargingPile = 51,
  kObstacleRawModelReflector = 52,
  kObstacleRawModelSingleFence = 53,
  kObstacleRawModelParkingLotGateOpen = 54,
  kObstacleRawModelParkingLotGateClose = 55
};

struct alignas(8) KpsClsRaw {
  std::uint8_t num_kps_cls_raw;
  std::uint8_t num_kps_cls_raw_name;
  std::array<std::int32_t, kMaxKpsClsRaw> id;
  std::array<float, kMaxKpsClsRaw> conf;
  std::array<std::array<std::int8_t, kMaxKpsClsRawName>, kMaxKpsClsRaw> name;
  int32_t type;
};

struct alignas(8) KeyPointsRaw {
  int32_t task_type;
  std::uint8_t num_key_points;
  std::uint8_t padding[4];
  std::array<KeyPoint, kMaxKeyPointsRaw> key_points;
};

enum class Global2peResultType : std::uint8_t {
  kGlobal2PeResultTypeWeather = 0,
  kGlobal2PeResultTypeLight = 1,
  kGlobal2PeResultTypeScene = 2,
  kGlobal2PeResultTypeTime = 3,
  kGlobal2PeResultTypeVanishPoint = 4,
  kGlobal2PeResultTypePosenet = 5,
  kGlobal2PeResultTypeBlockcls = 6,
  kGlobal2PeResultTypeBlurcls = 7,
  kGlobal2PeResultTypeGlarecls = 8,
  kGlobal2PeResultTypePrFromDepth = 9,
};

struct alignas(8) PRFromGround {
  float roll;
  float pitch;
  Global2peResultType type;
  std::uint8_t padding[7];
};

struct alignas(8) ObstacleRaw {
  Rect rect;
  float conf;
  int32_t model;
  int32_t source;
  std::uint8_t num_category;
  std::uint8_t num_property_bbox;
  std::uint8_t padding_1[2];
  std::array<Category, kMaxObstacleRawCategorySize> category;
  std::array<PropertyPerceptionBBox, kMaxPropertyBboxSize> property_bbox;

  std::array<KeyPoint, kKeyPointsSize> key_points;
  float depth;
  std::int32_t expand;

  Rect input_roi;
};

enum class KeyPointType : std::uint8_t {
  kColumnKps = 0,
  kSmallSodKps = 1,
  kWheelKps = 2
};

struct alignas(8) KeyPointsRawData {
  std::uint8_t num_key_points;
  KeyPointType type;
  std::uint8_t padding[6];
  std::array<KeyPoint, kMaxKeyPoints> key_points;
};

struct ObstacleRawsData {
  PerceptionHeader header;
  Category category;
  float conf_scale;
  std::uint8_t num_key_points_raws;
  std::uint8_t num_obstacle_raws;
  std::uint8_t padding[2];
  std::array<KeyPointsRawData, kStageOneKeyPointsSize> key_points_raws;
  std::array<ObstacleRaw, kMaxObstacleRaws> obstacle_raws;
};
}  // namespace autodrive
#endif