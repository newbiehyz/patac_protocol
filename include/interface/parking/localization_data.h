//
#ifndef AUTODRIVE_INTERFACE_LOCALIZATION_DATA_H_
#define AUTODRIVE_INTERFACE_LOCALIZATION_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/loc_odo.h"
#include "type/ndm_base.h"

namespace autodrive {

class LocalizationData final : public CommonMsgData {
 public:
  struct alignas(8) SpLocation {
    /// @brief Output odometry

    enum class SpLocationState : std::uint8_t {
      /// @brief 未初始化
      kUninit = 0,
      /// @brief 正在初始化
      kIniting = 1,
      /// @brief 定位正常
      kLocNormal = 2,
      /// @brief 定位不正常
      kLocUnnormal = 3,
      /// @brief 低精度定位
      kLocLowPrecision = 4,
    };
    Odometry loc_odom;
    /// @brief Status of the location
    SpLocationState state;
    std::uint8_t padding[7];
    /// @brief Velocity v
    Vector3f velocity;
    /// @brief Angular velocity
    Vector3f angular_vel;
  };

  struct alignas(8) Localization {
    /// @brief Header belong to ndm
    NdmSpec ndm_header;
    /// @brief Location info
    SpLocation location;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit LocalizationData(std::int64_t timestamp) noexcept;

  LocalizationData() noexcept;
  ~LocalizationData() noexcept final = default;
  LocalizationData(LocalizationData const&) = default;
  LocalizationData& operator=(LocalizationData const&) = default;
  LocalizationData(LocalizationData&&) = default;
  LocalizationData& operator=(LocalizationData&&) = default;

  /// @brief Get Localization data
  ///
  /// @return localization_data_
  Localization const& GetLocalization() const noexcept;

  /// @brief Mutable Localization data
  ///
  /// @return localization_data_
  Localization& MutableLocalization() noexcept;

  /// @brief Set Localization data
  ///
  /// @param localization_data localization data
  void SetLocalization(Localization const& localization_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  Localization localization_data_;
};

}  // namespace autodrive

#endif
