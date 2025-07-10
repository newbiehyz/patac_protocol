//
#ifndef AUTODRIVE_MODEL_COMMUNICATION_IMU_COMM_H_
#define AUTODRIVE_MODEL_COMMUNICATION_IMU_COMM_H_

#include <cstdint>

namespace autodrive {
namespace imu_comm {

// UTC time
struct alignas(8) UtcTime {
  /// @brief Year
  std::uint8_t year;
  /// @brief Month
  std::uint8_t month;
  /// @brief Day
  std::uint8_t day;
  /// @brief Hour
  std::uint8_t hour;
  /// @brief Minute
  std::uint8_t minute;
  /// @brief Second
  std::uint8_t second;
  /// @brief Millisecond
  std::uint16_t milli_second;
  /// @brief Time accuracy(ns)
  std::uint16_t time_accuracy;
  /// @brief  padding
  std::uint8_t padding[6];
};

// Imu
struct alignas(8) Imu {
  /// @brief Imu time
  UtcTime utc_time;
  /// @brief Heading(deg) (AngleHeading)
  float heading;
  /// @brief Pitch(deg) (AnglePitch)
  float pitch;
  /// @brief Roll(deg) (AngleRoll)
  float roll;
  /// @brief IMU raw acceleration(m/s/s) (AccelRawX, AccelRawY, AccelRawZ)
  float raw_acceleration_x;
  float raw_acceleration_y;
  float raw_acceleration_z;
  /// @brief IMU raw angular rate(deg/s) (AngRateRawX, AngRateRawY, AngRateRawZ)
  float raw_angular_rate_x;
  float raw_angular_rate_y;
  float raw_angular_rate_z;
  /// @brief padding
  std::uint8_t padding[4];
};

struct alignas(8) ImuComm {
  /// @brief high time stamp (s)
  std::uint32_t high_timestamp;
  /// @brief high time stamp (ns)
  std::uint32_t low_timestamp;
  /// @brief frame id
  std::uint32_t frame_id;
  /// @brief Padding
  std::uint8_t padding[4];
  /// @brief imu
  Imu imu;
};

}
}

#endif