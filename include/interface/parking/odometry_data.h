//
#ifndef AUTODRIVE_INTERFACE_ODOMETRY_DATA_H_
#define AUTODRIVE_INTERFACE_ODOMETRY_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/basic.h"

namespace autodrive {

class OdometryData final : public CommonMsgData {
 public:
  static constexpr std::uint8_t kWhlSize{4U};

  enum class OdometryDataSource : std::uint8_t {
    kTwoWheel = 1,
    kFourWheel = 2,
    kSpeed = 4,
    kYawRate = 8,
    kSteeringAngle = 16,
    kWheelSpeed = 32,
    kWheelPulse = 64,
    kGear = 128
  };

  enum class OdometryDataType : std::uint8_t {
    kCan = 0,
    kOfflineOptimized = 1,
    kOnlineIns = 2
  };

  enum class OdometryDataState : std::uint8_t {
    kNormal = 0,
    kUnnormal = 1,
    kReset = 2
  };

  enum class OdometryDataErrorCode : std::uint8_t {
    kNoError = 0,
    kErrorCanTimeout = 1,
    kErrorCanOdomOutrange = 2,
    kErrorCanHugeDiff = 3,
    kErrorCanAbnormalValue = 4,
    kErrorNoImu = 5,
    kErrorNoGnss = 6,
    kErrorNoCan = 7,
    kErrorNoImage = 8,
    kErrorGnssLost = 9,
    kErrorImageTimestamp = 10
  };

  enum class OdometryDataYawRateBiasEstimateState : std::uint8_t {
    kNormal = 0,
    kStaticUpdate = 1,
    kLaneUpdate = 2,
    kOnCurve = 3,
    kLaneNotSatisfied = 4
  };

  struct alignas(8) StableDataPitch {
    float pitch;
    bool is_valid;
    bool is_jitter;
    std::uint8_t padding[2];
  };

  enum class Gear : std::uint8_t {
    kGearTypeN = 0,
    KGearTypeP = 1,
    kGearTypeR = 2,
    kGearTypeD = 3
  };

  enum class LightIntensityValidity : std::uint8_t {
    kUndefinedDataAccuracy = 0,
    kTemporaryUndefinedData = 1,
    kDataAccuracyNotWithinSpecification = 2,
    kAccurateData = 3
  };

  enum class TurningLightStatus : std::uint8_t {
    kOff = 0,
    kOn = 1,
    kError = 2,
    kReserved = 3
  };

  enum class RainFallLevel : std::uint8_t {
    kAmntSnsrAmnt = 0,
    kAmntSnsrInitValue = 14,
    kAmntSnsrError = 15
  };

  struct alignas(8) Vehicle {
    /// @brief brightness of the light,raw
    std::uint16_t light_brightness_raw;
    /// @brief light intensity validity
    LightIntensityValidity lignt_validity;

    /// @brief turning light status
    TurningLightStatus left_turning_light_status;
    TurningLightStatus right_turning_light_status;

    /// @brief rain fall level
    RainFallLevel rain_fall_level;
    /// @brief Gear
    Gear gear;
    std::uint8_t padding[1];
    /// @brief Vehichle wheel speed
    std::array<float, kWhlSize> whl_speed;
    /// @brief Vehichle wheel pluse
    std::array<std::uint16_t, kWhlSize> whl_pulse;
    /// @brief Vehichle wheel pulse valid
    std::array<std::uint8_t, kWhlSize> whl_pulse_valid;

    /// @brief yaw rate valid
    uint8_t yaw_rate_valid;
    /// @brief speed valid
    uint8_t speed_valid;
    /// @brief gear valid
    uint8_t gear_valid;
    
    std::uint8_t wheel_size;
    /// @brief wheel pluse direction
    std::array<std::uint8_t, kWhlSize> wl_pluse_direction;
    /// @brief tire pressure valid
    std::array<std::uint8_t, kWhlSize> tire_pressure_valid; //0:invalid 1:valid
    /// @brief tire pressure
    std::array<float, kWhlSize> tire_pressure; //0:FL 1:FR 2:RL 3:RR
    
  };

  struct alignas(8) Odometry {
    /// @brief Position
    Vector3f position;
    /// @brief Orientation (roll, pitch, yaw)
    Vector3f orientation;
    /// @brief Speed (m/s), not raw data
    float speed;
    /// @brief Speed scale
    float speed_scale;
    /// @brief 2D acceleration (m/s^2)
    Vector2f acceleration;
    /// @brief Yaw rate (rad/s), not raw data
    float yaw_rate;
    /// @brief Yaw rate bias (rad/s)
    float yaw_rate_bias;
    /// @brief Wheel angle (deg)
    float wheel_angle;
    /// @brief Instantaneous slope (rad) ReservedFields
    float slope;
    /// @brief Stable pitch ReservedFields
    StableDataPitch stable_pitch;
    /// @brief Confidence of slope ReservedFields
    float slope_confidence;
    /// @brief Diagnostic code ReservedFields
    std::uint32_t diagnostic_code;
    /// @brief 3 covariances of translation (x,y,z) (m) ReservedFields
    Vector3d cov_translation;
    /// @brief 3 covariances of rotation (order: roll, pitch, yaw) (rad)
    /// ReservedFields
    Vector3d cov_eular_angle;
    /// @brief High resolution position ReservedFields
    Vector3d position_hr;
    /// @brief Odometry source bitmap
    std::uint16_t source;
    /// @brief Availability flag of slope ReservedFields
    bool is_slope_available;
    /// @brief Odometry type ReservedFields
    OdometryDataType type;
    /// @brief Odometry state
    OdometryDataState state;
    /// @brief Odometry error code ReservedFields
    OdometryDataErrorCode error_code;
    /// @brief Yaw rate bias estimate state ReservedFields
    OdometryDataYawRateBiasEstimateState yaw_rate_bias_estimate_state;
    std::uint8_t padding[1];
    /// @brief Vehichle sensor info
    Vehicle vehicle_info;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit OdometryData(std::int64_t timestamp) noexcept;

  OdometryData() noexcept;
  ~OdometryData() noexcept final = default;
  OdometryData(OdometryData const&) = default;
  OdometryData& operator=(OdometryData const&) = default;
  OdometryData(OdometryData&&) = default;
  OdometryData& operator=(OdometryData&&) = default;

  /// @brief Get Odometry data
  ///
  /// @return odometry_data_
  Odometry const& GetOdometry() const noexcept;

  /// @brief Mutable Odometry data
  ///
  /// @return odometry_data_
  Odometry& MutableOdometry() noexcept;

  /// @brief Set Odometry data
  ///
  /// @param odometry_data odometry_data
  void SetOdometry(Odometry const& odometry_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  Odometry odometry_data_;
};

}  // namespace autodrive

#endif
