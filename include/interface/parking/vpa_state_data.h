//
#ifndef AUTODRIVE_INTERFACE_VPA_STATE_DATA_H_
#define AUTODRIVE_INTERFACE_VPA_STATE_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/basic.h"

namespace autodrive {

class VpaStateData final : public CommonMsgData {
 public:
  enum class MapDeletingStage : std::uint8_t {
    kDefault = 0,
    kDeleting = 1,
    kDeleteSucessful = 2,
    kDeleteFailed = 3
  };

  enum class MapSavingStage : std::uint8_t {
    kDefault = 0,
    kSaving = 1,
    kSaveSucessful = 2,
    kSaveFailed = 3
  };

  /// @brief 建图状态
  enum class MappingStage : std::uint8_t {
    kDefaultMappingStage = 0,         // 默认状态
    kMapping = 1,         // 正在进行建图
    kMappingSuccess = 2,  // 建图成功
    kMappingFailed = 3    // 建图失败
  };

  /// @brief 地图优化状态
  enum class MapOptimizeStage : std::uint8_t {
    kDefaultMapOptimizeStage = 0,          // 默认状态
    kOptimizing = 1,       // 优化中
    kOptimizeSuccess = 2,  // 优化成功
    kOptimizeFailed = 3    // 优化失败
  };

  enum class MappingOddType : std::uint8_t {
    kDefault = 0,
    kMappingOddOk = 1,
    kMappingOddNokNotIndoor = 2,
    kMappingOddNokNotUndergroundPark = 3,
    kMappingOddNokSpeedOutOfLimit = 4,
    kMappingOddNokNoSemanticInfo = 5,
    kMappingOddNokNotEnoughSemanticInfo = 6,
    kMappingOddNokLightnessNok = 7,
    kMappingOddNokRainFallNok = 8,
    kMappingOddNokSlopeOutOfLimit = 9,
    kMappingOddNokPreConditionCheck = 10,
    kMappingOddNokReserved = 11
  };

  enum class LocOddType : std::uint8_t {
    kDefault = 0,
    kLocOddOk = 1,
    kLocOddNokNotIndoor = 2,
    kLocOddNokNotUndergroundPark = 3,
    kLocOddNokSpeedOutOfLimit = 4,
    kLocOddNokNoSemanticInfo = 5,
    kLocOddNokNotEnoughSemanticInfo = 6,
    kLocOddNokLightnessNok = 7,
    kLocOddNokRainFallNok = 8,
    kLocOddNokSlopeOutOfLimit = 9,
    kLocOddNokPreConditionCheck = 10,
    kLocOddNokReserved = 11
  };

  enum class LocalLocStage : std::uint8_t {
    kNO = 0,
    kOngoing = 1,
    kSuccess = 2
  };

  enum class ErrorCode : std::uint16_t {
    kNoError = 0,

    /// @brief system error
    kErrorInsufficientDiskFreespace = 1,  // 磁盘内存不足

    /// @brief input data error
    kErrorTsReset = 1000,        // 时间戳乱序或者时间戳间隔太长
    kErrorNoPerception = 1001,   // 长时间没有感知数据输入
    kError3dOdoAbnormal = 1002,  // 3d odo值异常
    kErrorOdomError = 1003,      // odom重置/异常
    kErrorNoSyncGnss = 1004,     // 没有找到同步的gnss信号
    kErrorInvalidGnss = 1005,    // GNSS信号无效

    /// @brief map error
    kErrorWaitForSlotTooLong = 2000,            // 等待目标车位过久
    kErrorOutdoorInMapping = 2001,              // 建图时开出室外
    kErrorMapUnsuitableForLocalization = 2002,  // 地图不适合用来定位
    kErrorMapOptimizationTimeOut = 2003,        // 优化时间过长
    kErrorMappingDistTooLong = 2004,            // 学习距离过长(>1.5km)
    kErrorMappingDistTooShort = 2005,           // 学习距离过短(<15m)
    kErrorMapPlanningFailed = 2006,     // 地图全局路经规划失败
    kErrorMapSaveFailed = 2007,         // 地图保存失败
    kErrorMappingCrossFloor = 2008,     // 建图过程中跨层
    kErrorMapLoopClosure = 2009,        // 建图闭环
    kErrorDeleteNotExistMap = 2010,     // 删除不存在的地图
    kErrorOverSpeed = 2011,             // 车速超过20km/h
    kErrorOverSpeedTimeOut = 2012,      // 车速超过25km/h超时(5s)
    kErrorOverSlope = 2013,             // 坡度超限
    kErrorGearRDistanceTooLong = 2014,  // 倒挡距离太长(>2m)
    kErrorNoAvaliableMapFound = 2015,   // 附近无可用的地图
    kErrorNotInParkingSlot = 2016,      // 车辆没有停在线车位内

    /// @brief loc error
    kErrorLocDeviationFromNavigation = 3000,  // 定位偏离导航轨迹
    kErrorMapMatchingError = 3001,            // 地图匹配错误
    kErrorOutOfMap = 3002,                    // 超出地图范围
    kErrorMissMapKeyElements = 3003,          // 地图关键要素缺失
    kErrorPoseOdomInconsistency = 3004,  // 定位矫正的位姿和ODO一致性存在问题
    kErrorMapError = 3005,  // 地图错误，地图结构和数据缺少或者错误
    kErrorNoMap = 3006,         // 没有地图
    kErrorMapFileError = 3007,  // 地图文件错误

  };

  enum class HPAState : std::uint8_t {
    kDefault = 0,
    /// @brief 系统待机状态
    kOff = 1,
    /// @brief 巡航前匹配地图状态
    kStandby = 2,
    /// @brief 路径学习状态
    kPathLearning = 3,
    /// @brief 巡航状态
    kCruising = 4,
    /// @brief 巡航前置条件检测状态(要求车辆驶出地库，并且在平地行驶200m)
    kPreConditionCheck = 5,
  };

  enum class VPAEventType : std::uint8_t {
    kDefault = 0,
    /// @brief Send this event when error occurred , check ErrorCode for details
    kErrorEvent = 1,
    /// @brief Send this event when request mapping
    kRequestMappingEvent = 2,
    /// @brief Send this event when request location
    kRequestLocalizationEvent = 3,
    /// @brief Loc odd event, check LocOddType for details
    kLocOddEvent = 4,
    /// @brief Mapping odd event, check MappingOddType for details
    kMappingOddEvent = 5,
    /// @brief HPAState event, check HPAState for details
    kHpaStateEvent = 6,
    /// @brief MapSavingStage event, check MapSavingStage for details
    kMapSavingStageEvent = 7,
    /// @brief Local loc stage event, check LocalLocStage for details
    kLocalLocStageEvent = 8,
    /// @brief First time mapping event
    kFirstTimeMappingEvent = 9,
    /// @brief First time cur parking mapping event
    kFirstTimeCurParkingMappingEvent = 10,
    /// @brief Find available map, check avaliable_map_id for details
    kFindAvailableMapEvent = 11,
    /// @brief Delete Map Stage, check MapDeletingStage for details
    kMapDeleteStageEvent = 12,
    /// @brief Enter under ground parking event
    kEnterUndergroundParkingEvent = 13,
    /// @brief In parking slot event
    kInParkingSlotEvent = 14,
    /// @brief Mapping stage event, check MappingStage for details
    kMappingStageEvent = 15,
    /// @brief Mapping optimize event,check MapOptimizeStage for details
    kMappingOptimizeStageEvent = 16,
    /// @brief out under ground parking event
    kOutUndergroundParkingEvent = 17,
    /// @brief find valid gnss event
    kFindValidGNSSEvent = 18,
  };

  struct alignas(8) VpaState {
    /// @brief Should always check this first,only update state variables
    /// relevant to the event type each time
    VPAEventType vpa_event;
    /// @brief state variables
    HPAState hpa_state;
    MappingOddType mapping_odd_type;
    LocOddType loc_odd_type;
    MapSavingStage map_saving_stage;
    LocalLocStage local_loc_stage;
    MapDeletingStage map_deleted_stage;
    MappingStage map_stage;
    MapOptimizeStage map_optimized_stage;
    ErrorCode error_code;
    std::uint8_t padding[2];
    /// @brief Used  When hpa_state==VPAEventType::kInParkingSlotEvent, if
    /// target_parking_slot_id < 0,means the car not in parking slot else it's
    /// the parking slot id car parking in.
    int32_t target_parking_slot_id;
    /// @brief Used When hpa_state==VPAEventType::kMapSavingStageEvent, [0-100]
    uint32_t map_saving_progress;
    /// @brief Used When hpa_state==VPAEventType::kFindAvailableMapEvent
    std::uint32_t avaliable_map_id;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit VpaStateData(std::int64_t timestamp) noexcept;

  VpaStateData() noexcept;
  ~VpaStateData() noexcept final = default;
  VpaStateData(VpaStateData const&) = default;
  VpaStateData& operator=(VpaStateData const&) = default;
  VpaStateData(VpaStateData&&) = default;
  VpaStateData& operator=(VpaStateData&&) = default;

  /// @brief Get Vpa State
  ///
  /// @return vpa_state_
  VpaState const& GetVpaStateData() const noexcept;

  /// @brief Mutable Vpa State
  ///
  /// @return vpa_state_
  VpaState& MutableVpaStateData() noexcept;

  /// @brief Set Vpa State
  ///
  /// @param vpa_state Vpa State
  void SetVpaStateData(VpaState const& vpa_state) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  VpaState vpa_state_;
};

}  // namespace autodrive

#endif
