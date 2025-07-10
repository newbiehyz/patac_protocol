//
#ifndef AUTODRIVE_TYPE_NDM_BASE_H_
#define AUTODRIVE_TYPE_NDM_BASE_H_

#include <array>
#include <cstdint>

#include <type/basic.h>

namespace autodrive {

static constexpr std::uint32_t kMaxNdmHeaderVersionSize{8U};
static constexpr std::uint32_t kMaxNdmHeaderDateSize{16U};
static constexpr std::uint32_t kMaxNdmHeaderProjectionSize{8U};
static constexpr std::uint32_t kMaxNdmHeaderUtmZoneSize{8U};
static constexpr std::uint32_t kMaxIdNameSize{32U};
static constexpr std::uint32_t kMaxTimeLimits{4U};
static constexpr std::uint32_t kMaxVehicleTypesSize{4U};

enum class DrivingSide : std::uint8_t {
  kRightHandDriving = 0,
  kLeftHandDriving = 1
};

struct alignas(8) NdmSpec {
  /// @brief Version
  char version[kMaxNdmHeaderVersionSize];
  /// @brief Date
  char date[kMaxNdmHeaderDateSize];
  /// @brief Time stamp
  std::uint64_t timestamp;
  /// @brief Sequence id
  std::uint64_t seq_id;
  /// @brief Projection tyep e.g. WGS-84/UTM ...
  char projection[kMaxNdmHeaderProjectionSize];
  /// @brief UTM ZONE
  char utm_zone[kMaxNdmHeaderUtmZoneSize];
  /// @brief Offset of the beginning base
  Vector3d base_offset;
};

struct alignas(8) Id {
  std::uint32_t id;
  std::uint8_t num_name;
  std::uint8_t padding[7];
  std::array<char, kMaxIdNameSize> name;
};

/// @breif Lane
struct alignas(8) Link {
  Id id;
  float offset;
  float end_offset;
};

struct alignas(8) VehicleType {
  enum class Type : std::uint8_t {
    kUnknown = 0,
    kCar = 1,
    kMotorcycle = 2,
    kBicycle = 3,
    kTruck = 4,
    kTaxi = 5,
    kBus = 6,
    kPedestrian = 7,
    kCableCar = 8
  };
  Type type;
  std::uint8_t padding[3];
  float conf;
};

enum class SpeedLimitType : std::uint8_t { kUnknown = 0, kMin = 1, kMax = 2 };

enum class SpeedLimitSource : std::uint8_t {
  kUnknown = 0,
  kImplicit = 1,
  kExplicit = 2
};

struct alignas(8) SpeedLimit {
  SpeedLimitType limit_type;
  SpeedLimitSource source;
  float speed_value;
  std::uint8_t padding[2];
  float offset;
  float end_offset;
};

struct alignas(8) Number {
  float value;
  float cov;
};

struct alignas(8) TimeLimit {
  float time_begin;
  float time_end;
};

struct alignas(8) ParkingSpaceRestriction {
  Number number_limit;
  std::uint8_t num_time_limits;
  std::uint8_t num_vehicle_types;
  std::uint8_t padding[6];
  std::array<TimeLimit, kMaxTimeLimits> time_limits;
  std::array<VehicleType, kMaxVehicleTypesSize> vehicle_types;
};

struct alignas(8) Index {
  std::uint64_t x;
  std::uint64_t y;
  std::uint64_t z;
};

}  // namespace autodrive

#endif
