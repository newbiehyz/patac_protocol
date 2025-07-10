//
#ifndef AUTODRIVE_INTERFACE_FUSION_PARKING_SLOTS_DATA_H_
#define AUTODRIVE_INTERFACE_FUSION_PARKING_SLOTS_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/parking_slot.h"

namespace autodrive {

class FusionParkingSlotsData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxParkingSlotsSize{20U};

  struct alignas(8) FusionParkingSlots {
    std::uint32_t num_parking_slots;
    std::uint32_t num_certificated_slots;
    std::array<ParkingSlotInfo, kMaxParkingSlotsSize> parking_slots;
    std::array<std::int32_t, kMaxParkingSlotsSize> certified_slots;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit FusionParkingSlotsData(std::int64_t timestamp) noexcept;

  FusionParkingSlotsData() noexcept;
  ~FusionParkingSlotsData() noexcept final = default;
  FusionParkingSlotsData(FusionParkingSlotsData const&) = default;
  FusionParkingSlotsData& operator=(FusionParkingSlotsData const&) = default;
  FusionParkingSlotsData(FusionParkingSlotsData&&) = default;
  FusionParkingSlotsData& operator=(FusionParkingSlotsData&&) = default;

  /// @brief Get Fusion Parking Slots data
  ///
  /// @return fusion_parking_slots_data_
  FusionParkingSlots const& GetFusionParkingSlots() const noexcept;

  /// @brief Mutable Fusion Parking Slots data
  ///
  /// @return fusion_parking_slots_data_
  FusionParkingSlots& MutableFusionParkingSlots() noexcept;

  /// @brief Set Fusion Parking Slots data
  ///
  /// @param fusion_parking_slots_data fusion parking slots data
  void SetFusionParkingSlots(FusionParkingSlots const& fusion_parking_slots_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  FusionParkingSlots fusion_parking_slots_data_;
};

}  // namespace autodrive

#endif
