//
#ifndef AUTODRIVE_INTERFACE_HMI_MAP_PARKING_SLOTS_DATA_H_
#define AUTODRIVE_INTERFACE_HMI_MAP_PARKING_SLOTS_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/basic.h"

namespace autodrive {

class HmiMapParkingSlotsData final : public CommonMsgData {
 public:
  static constexpr uint32_t kMaxSlotNum = 1024;
  static constexpr uint32_t kPointNum = 4;

  // 建图完成后，缩略地图中的车位信息
  struct MapParkingSlot {
    int32_t slot_id;                     // 车位id
    uint8_t slot_state;                  // 车位状态全部为0-未知状态
    uint8_t slot_type;                   // 车位类型
    uint8_t padding[2];                  // padding
    std::array<Vector3f, kPointNum> slot_points; // 车位四个角点
  };
  // 缩略地图全部车位信息
  struct MapParkingSlots {
    uint32_t slot_num;
    uint8_t padding[4];
    std::array<MapParkingSlot, kMaxSlotNum> all_map_parking_slots;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit HmiMapParkingSlotsData(std::int64_t timestamp) noexcept;

  HmiMapParkingSlotsData() noexcept;
  ~HmiMapParkingSlotsData() noexcept final = default;
  HmiMapParkingSlotsData(HmiMapParkingSlotsData const&) = default;
  HmiMapParkingSlotsData& operator=(HmiMapParkingSlotsData const&) = default;
  HmiMapParkingSlotsData(HmiMapParkingSlotsData&&) = default;
  HmiMapParkingSlotsData& operator=(HmiMapParkingSlotsData&&) = default;

  /// @brief Get Map Display Data
  ///
  /// @return map_display_
  MapParkingSlots const& GetMapParkingSlots() const noexcept;

  /// @brief Mutable Sync Map Display Data
  ///
  /// @return map_display_
  MapParkingSlots& MutableMapParkingSlots() noexcept;

  /// @brief Set Sync Map Display Data
  ///
  /// @param map_display sync cMap Display Data
  void SetMapParkingSlots(MapParkingSlots const& map_parking_slots) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  MapParkingSlots map_parking_slots_;
};

}  // namespace autodrive

#endif
