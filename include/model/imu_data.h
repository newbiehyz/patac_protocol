//
#ifndef AUTODRIVE_MODEL_IMU_DATA_H_
#define AUTODRIVE_MODEL_IMU_DATA_H_

#include <string.h>
#include <cstdint>
#include <memory>

#include <foundation/type/common_msg_data.h>
#include "model/imu_comm.h"

namespace autodrive {

// Imu data
class ImuData final : public CommonMsgData {
 public:

  // Imu
  struct alignas(8) Imu {
    /// @brief Imu time
    std::uint64_t utc_time;
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

  ImuData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ImuData(std::int64_t timestamp) noexcept;
  ~ImuData() noexcept final = default;
  ImuData(ImuData const&) = default;
  ImuData& operator=(ImuData const&) = default;
  ImuData(ImuData&&) = default;
  ImuData& operator=(ImuData&&) = default;

  /// @brief Get Imu
  ///
  /// @return imu_
  Imu const& GetImu() const noexcept;

  /// @brief Get mutable Imu
  ///
  /// @return imu_
  Imu& MutableImu() noexcept;

  /// @brief Set Imu
  ///
  /// @param Imu  imu_
  void SetImu(Imu const& imu) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

  // /// @brief fill data
  // ///
  // /// @param comm_ptr gnss comm ptr
  // void FillData(imu_comm::ImuComm* comm_ptr) noexcept(false);

  // /// @brief Fill communication data buffer
  // ///
  // /// @param data_buf  allocated data buffer
  // /// @param nbytes  max length to fill
  // ///
  // /// @return data length
  // std::uint32_t FillCommData(void* data_buf, std::uint32_t nbytes) noexcept(false);

 private:
  /// @brief Imu
  Imu imu_;
};

}

#endif