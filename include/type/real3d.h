//
#ifndef AUTODRIVE_TYPE_REAL3D_H_
#define AUTODRIVE_TYPE_REAL3D_H_

#include <type/basic.h>
#include "perception_base.h"
#include "dynamic_obstacle.h"
namespace autodrive {

static constexpr std::uint32_t kReal3DResultsSize{3U};
static constexpr std::uint32_t kMaxReal3DResultSize{64U};
static constexpr std::uint32_t kMaxCoordinateTypeSize{64U};

enum class Real3DType : std::uint8_t {
  kPedestrian = 0,
  kVehicle = 1,
  kCyclist = 2
};

enum class Roi3DMapType : std::uint8_t {
  kPtUndistortMap = 0,
  kPtDistortMap = 1,
  kEqflUMap = 2,
  kEqflVMap = 3
};

struct alignas(8) Real3DResult {
  DynamicObstacle bev_obstacle;
  Box3D img_3dbox;
  Rect img_rect;
};

struct alignas(8) Real3DResults {
  PerceptionHeader header;
  Real3DType type;
  std::uint8_t num_coordinate_type;
  std::uint8_t num_real3d_result;
  std::uint8_t padding[6];
  std::array<char, kMaxCoordinateTypeSize> coordinate_type;
  std::array<Real3DResult, kMaxReal3DResultSize> real3d_result;
};

struct alignas(8) Real3DData {
  PerceptionHeader header;
  std::array<Real3DResults, kReal3DResultsSize> real3d_results;
};

}  // namespace autodrive

#endif