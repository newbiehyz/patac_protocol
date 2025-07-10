//
#ifndef AUTODRIVE_TYPE_BASE_H_
#define AUTODRIVE_TYPE_BASE_H_

#include <cstdint>

namespace autodrive {

struct alignas(8) Vector2f {
  /// @brief x
  float x;
  /// @brief y
  float y;
};

struct alignas(8) Vector2d {
  /// @brief x
  double x;
  /// @brief y
  double y;
};

struct alignas(8) Vector2i {
  /// @brief x
  std::int32_t x;
  /// @brief y
  std::int32_t y;
};

struct Vector2u8Nona {
  /// @brief x
  std::uint8_t x;
  /// @brief y
  std::uint8_t y;
};

struct Vector2u16Nona {
  /// @brief x
  std::uint16_t x;
  /// @brief y
  std::uint16_t y;
};

struct Vector2i8Nona {
  /// @brief x
  std::int8_t x;
  /// @brief y
  std::int8_t y;
};

struct Vector2i16Nona {
  /// @brief x
  std::int16_t x;
  /// @brief y
  std::int16_t y;
};

struct Vector3u8Nona {
  /// @brief x
  std::uint8_t x;
  /// @brief y
  std::uint8_t y;
  /// @brief z
  std::uint8_t z;
};

struct Vector3u16Nona {
  /// @brief x
  std::uint16_t x;
  /// @brief y
  std::uint16_t y;
  /// @brief z
  std::uint16_t z;
};

struct Vector3i8Nona {
  /// @brief x
  std::int8_t x;
  /// @brief y
  std::int8_t y;
  /// @brief z
  std::int8_t z;
};

struct Vector3i16Nona {
  /// @brief x
  std::int16_t x;
  /// @brief y
  std::int16_t y;
  /// @brief z
  std::int16_t z;
};
struct alignas(8) Vector3f {
  /// @brief x/roll
  float x;
  /// @brief y/pitch
  float y;
  /// @brief z/yaw
  float z;
  /// @brief padding
  std::uint8_t padding[4];
};

struct alignas(8) Vector3d {
  /// @brief x/roll
  double x;
  /// @brief y/pitch
  double y;
  /// @brief z/yaw
  double z;
};

struct alignas(8) Vector4f {
  /// @brief w for quaternion
  float w;
  /// @brief x for quaternion
  float x;
  /// @brief y for quaternion
  float y;
  /// @brief z for quaternion
  float z;
};

struct alignas(8) Vector4d {
  /// @brief w for quaternion
  double w;
  /// @brief x for quaternion
  double x;
  /// @brief y for quaternion
  double y;
  /// @brief z for quaternion
  double z;
};

}

#endif
