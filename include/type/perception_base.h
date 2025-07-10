//
#ifndef AUTODRIVE_TYPE_PERCEPTION_BASE_H_
#define AUTODRIVE_TYPE_PERCEPTION_BASE_H_

#include <array>
#include <cstdint>

#include <type/basic.h>

namespace autodrive {

static constexpr std::uint32_t kMaxCameraMatrixMatGnd2ImgSize{9U};
static constexpr std::uint32_t kMaxCameraMatrixMatImg2GndSize{9U};
static constexpr std::uint32_t kMaxCameraMatrixMatVcsgnd2ImgSize{9U};
static constexpr std::uint32_t kMaxCameraMatrixMatImg2VcsgndSize{9U};
static constexpr std::uint32_t kMaxCameraMatrixMatLocal2ImgSize{9U};
static constexpr std::uint32_t kMaxCameraMatrixMatImg2LocalSize{9U};
static constexpr std::uint32_t kMaxCameraMatrixMatVcs2ImgSize{12U};
static constexpr std::uint32_t kPropertyNameSize{50U};
static constexpr std::uint32_t kMaxPolygonSize{8U};

struct alignas(8) Rect {
  float left;
  float top;
  float right;
  float bottom;
};

struct alignas(8) RectInt {
  int16_t left;
  int16_t top;
  int16_t right;
  int16_t bottom;
};

struct alignas(8) BbRect {
  int16_t x;
  int16_t y;
  int16_t width;
  int16_t height;
};

struct alignas(8) Polygon {
  std::uint32_t num_points;
  std::uint8_t padding[4];
  std::array<Vector3f, kMaxPolygonSize> points;
};

/// @brief Obstacle info in img
struct alignas(8) ImageSpace {
  /// @brief Static
  /// A rect.l => xLeftCoord, rect.r => xRightCoord
  /// A rect.t => yTopCoord, rect.b => yBottomCoord
  RectInt rect;
};

struct alignas(8) CameraMatrix {
  std::array<float, kMaxCameraMatrixMatGnd2ImgSize> mat_gnd2img;
  std::array<float, kMaxCameraMatrixMatImg2GndSize> mat_img2gnd;
  std::array<float, kMaxCameraMatrixMatVcsgnd2ImgSize> mat_vcsgnd2img;
  std::array<float, kMaxCameraMatrixMatImg2VcsgndSize> mat_img2vcsgnd;
  std::array<float, kMaxCameraMatrixMatLocal2ImgSize> mat_local2img;
  std::array<float, kMaxCameraMatrixMatImg2LocalSize> mat_img2local;
  std::array<float, kMaxCameraMatrixMatVcs2ImgSize> mat_vcs2img;
};

struct alignas(8) RoiMap {
  Rect model_roi;
  std::int32_t model_id;
  std::uint8_t padding[4];
};

struct alignas(8) PerceptionHeader {
  std::int32_t cam_idx;
  std::int64_t frame_id;
  std::int64_t time_stamp;
};

struct alignas(8) Category {
  /// @brief E.g. weather
  std::int32_t property_type;
  /// @brief Channel index of category result
  std::int32_t property;
  float property_conf;
  /// @brief Category name of specific channel
  char property_name[kPropertyNameSize];
};

/// @brief Image id
enum class ImageId : std::uint8_t {
  kCylinderRear = 0,
  kCylinderFront = 1,
  kCylinderLeft = 2,
  kCylinderRight = 3,
  kIpm = 4,
  kRear = 5,
  kFront = 6,
  // Count should be written in the last place.
  kImageIDCount
};

enum class RoiId : std::uint8_t {
  kNone = 0,
  kObstacle1 = 1,  // 704, 384
  kObstacle2 = 2,  // 704, 384
  kLane1 = 3,      // 1024, 288
  kLane2 = 4       // 1024, 288
};

enum class VehProperty: std::uint8_t{
  kVehPropertyType = 0,
  kVehPropertyClassification = 1,
  kVehPropertyLight = 2,
  kVehProperty3DBox = 3,
  kVehPropertyKeyPoints = 4,
  kVehPropertyVehBBox = 5,
  kVehPropertyKeyPointsConf = 6,
  kVehPropertyBigSmallCls = 7,
  kVehPropertyPoliceLight = 8,
  kVehPropertyOcclusion = 9,
  kVehPropertyPose = 10,
  kVehPropertyVehFull = 11,
  kVehPropertyOritationCls = 12,
  kVehPropertyOritationRe = 13,
  kVehProperty12KeyPoints = 14,
  kVehProperty12KeyPointsConf = 15,
  kVehproperty12KeyPointsCls = 16,
  kVehPropertyTruncation = 17,
  kVehPropertyCnt = 18
};

enum class PedProperty : std::uint8_t{
  kPedPropertyOrientation = 0,
  kPedPropertyCyclist = 1,
  kPedPropertyAge = 2,
  kPedPropertyPose = 3,
  kPedPropertyPosNeg = 4,
  kPedPropertyBBox = 5,
  kPedPropertyOcclusion = 6,
  kPedPropertyPosNegOcc = 7,
  kPedProperty2PEDet = 8,
  kPedPropertyCnt = 9
};

enum class VehRearProperty : std::uint8_t{
  kVehRearPropertyOcclusion = 0,
  kVehRearPropertyPart = 1,
  kVehRearPropertyBBox = 2,
  kVehRearPropertyLmks3 = 3,
  kVehRearPropertyScaleRe = 4,
  kVehRearPropertyPlate = 5,
  kVehRearPropertyCnt = 6
};

struct alignas(8) Box3D {
  Vector3f lower_lt;
  Vector3f lower_lb;
  Vector3f lower_rb;
  Vector3f lower_rt;
  Vector3f upper_lt;
  Vector3f upper_lb;
  Vector3f upper_rb;
  Vector3f upper_rt;
  float conf;
};

enum class SceneRegionType : std::uint8_t {
    kUnknown = 0,
    kIndoor = 1,
    kOutdoor = 2,
};

enum class SceneIlluminationType : std::uint8_t {
  kUnknown = 0,
  kDark = 1,
  kBright = 2,
};

enum class SceneParkingType : std::uint8_t {
  kUnknown = 0,
  kSlot = 1,
  kMechanicalSlot = 2,
};

struct alignas(8) SceneParsing{
  ImageId image_id;
  SceneRegionType region_type;
  SceneIlluminationType illu_type;
  SceneParkingType parking_type;
  float region_conf;
  float illu_conf;
  float parking_conf;
};

}  // namespace autodrive

#endif
