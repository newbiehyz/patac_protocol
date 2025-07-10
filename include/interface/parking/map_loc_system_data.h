//
#ifndef AUTODRIVE_INTERFACE_MAP_LOC_SYSTEM_DATA_H_
#define AUTODRIVE_INTERFACE_MAP_LOC_SYSTEM_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>
#include "type/system_type_base.h"

namespace autodrive {

class MapLocSystemData final : public CommonMsgData {
 public:
  enum class AVP_LPState : std::uint8_t {
    AVPOff = 0,              // 关闭
    kAbort = 1,            // 退出
    kStandby = 2,          // 待机
    kMapLearning = 3,      // 开始地图学习
    kParkinPreActive = 4,  // 巡航准备
    kCruising = 5,         // 开始巡航
    kParkInProcess = 6,    // 开始泊车中
    kSuspend = 7,          // 暂停
    kFunctionComplete = 8,  // 泊车功能完成
    kMapManageMent = 9     // 管理地图
  };

  enum class ControlCmd : std::uint8_t {
    kDefault = 0,
    /// @brief Get all the map headers
    kGetAllMapHeader = 1,
    /// @brief Get one map info
    kGetOneMapInfo = 2,
    /// @brief delete one map
    kDeleteOneMap = 3,
    /// @brief delete all maps
    kDeleteAllMap = 4,
    /// @brief 发送目标车位
    kSendTargetParkingSlot = 5,
    /// @brief 完成地图学习
    kFinishMapLearning = 6,
    /// @brief 到达巡航终点
    kCruisingEnd = 7,
    /// @brief Reset Maploc Module
    kReset = 8
  };


  struct alignas(8) MapLocSystem {
    /// @brief apv lp state
    AVP_LPState avp_lp_state;
    /// @brief control cmd
    ControlCmd control_cmd;
    /// @brief Id of map to be deleted
    std::uint8_t delete_map_id;

    std::uint8_t padding;

    /// @brief Id of target parking slot, used when ControlCmd ==
    /// kSendTargetParkingSlot
    std::uint32_t target_parking_slot_id;
    /// @brief Id of map to be deleted or get map info
    std::uint32_t select_map_id;

    std::uint8_t padding_1[4];
    ApaStateInfo apa_states;
  };

  
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit MapLocSystemData(std::int64_t timestamp) noexcept;

  MapLocSystemData() noexcept;
  ~MapLocSystemData() noexcept final = default;
  MapLocSystemData(MapLocSystemData const&) = default;
  MapLocSystemData& operator=(MapLocSystemData const&) = default;
  MapLocSystemData(MapLocSystemData&&) = default;
  MapLocSystemData& operator=(MapLocSystemData&&) = default;

  /// @brief Get MapLocSystem data
  ///
  /// @return map_loc_system_data_
  MapLocSystem const& GetMapLocSystem() const noexcept;

  /// @brief Mutable MapLocSystem data
  ///
  /// @return map_loc_system_data_
  MapLocSystem& MutableMapLocSystem() noexcept;

  /// @brief Set MapLocSystem data
  ///
  /// @param map_loc_system_data map loc system data
  void SetMapLocSystem(MapLocSystem const& map_loc_system_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  MapLocSystem map_loc_system_data_;
};

}  // namespace autodrive

#endif
