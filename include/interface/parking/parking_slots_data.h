//
#ifndef AUTODRIVE_INTERFACE_PARKING_SLOTS_DATA_H_
#define AUTODRIVE_INTERFACE_PARKING_SLOTS_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/parking_slot.h"

namespace autodrive {

class ParkingSlotsData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxParkingSlotsSize{32U};

  struct alignas(8) ParkingSlots {
    std::uint32_t num_parking_slots;
    std::uint8_t padding[4];
    std::array<ParkingSlotInfo, kMaxParkingSlotsSize> parking_slots;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ParkingSlotsData(std::int64_t timestamp) noexcept;

  ParkingSlotsData() noexcept;
  ~ParkingSlotsData() noexcept final = default;
  ParkingSlotsData(ParkingSlotsData const&) = default;
  ParkingSlotsData& operator=(ParkingSlotsData const&) = default;
  ParkingSlotsData(ParkingSlotsData&&) = default;
  ParkingSlotsData& operator=(ParkingSlotsData&&) = default;

  /// @brief Get Parking Slots data
  ///
  /// @return parking_slots_data_
  ParkingSlots const& GetParkingSlots() const noexcept;

  /// @brief Mutable Parking Slots data
  ///
  /// @return parking_slots_data_
  ParkingSlots& MutableParkingSlots() noexcept;

  /// @brief Set Parking Slots data
  ///
  /// @param parking_slots_data parking slots data
  void SetParkingSlots(ParkingSlots const& parking_slots_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  ParkingSlots parking_slots_data_;
};

}  // namespace autodrive

#endif
