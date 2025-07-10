//
#ifndef AUTODRIVE_INTERFACE_USS_TRACKING_MODE_DATA_H_
#define AUTODRIVE_INTERFACE_USS_TRACKING_MODE_DATA_H_

#include <foundation/type/common_msg_data.h>
#include "type/uss_base.h"
#include "uss_parking_slots_data.h"

namespace autodrive {

class UssTrackingModeData final : public CommonMsgData {
 public:
  struct alignas(8) UssTrackingMode {
    PlvState plv_mode;
    PsuState psu_mode;
    std::uint8_t uss_ps_clear_distance;
    OdoState odo_mode;
    std::uint8_t padding[4];
    UssParkingSlotsData::PerFusUssPs ps_correct;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit UssTrackingModeData(std::int64_t timestamp) noexcept;

  UssTrackingModeData() noexcept;
  ~UssTrackingModeData() noexcept final = default;
  UssTrackingModeData(UssTrackingModeData const&) = default;
  UssTrackingModeData& operator=(UssTrackingModeData const&) = default;
  UssTrackingModeData(UssTrackingModeData&&) = default;
  UssTrackingModeData& operator=(UssTrackingModeData&&) = default;

  /// @brief Get Uss Tracking Mode data
  ///
  /// @return uss_tracking_mode_data_
  UssTrackingMode const& GetUssTrackingMode() const noexcept;

  /// @brief Mutable Uss Tracking Mode data
  ///
  /// @return uss_tracking_mode_data_
  UssTrackingMode& MutableUssTrackingMode() noexcept;

  /// @brief Set Uss Tracking Mode data
  ///
  /// @param fusion_result_data Uss Tracking Mode data
  void SetUssTrackingMode(UssTrackingMode const& uss_tracking_mode_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  UssTrackingMode uss_tracking_mode_data_;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_USS_TRACKING_MODE_DATA_H_