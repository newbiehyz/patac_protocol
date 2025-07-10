//
#ifndef AUTODRIVE_INTERFACE_FUSION_DEBUG_DATA_H_
#define AUTODRIVE_INTERFACE_FUSION_DEBUG_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/system_type_base.h"

namespace autodrive {

class VisionSystemData final : public CommonMsgData {
 public:
  /// @brief Parking or driving
  enum class VisionSystemStatus : std::uint8_t {
    kDriving = 0,
    kParking = 1,
    kSleeping = 2
  };

  enum class CalibrationStatus : std::uint8_t {
    kNone = 0,
    kStartEol = 1,
    kStartOnl = 2,
    kPauseOnl = 3,
    kResumeOnl = 4,
    kStopOnl = 5,
    kStopEol = 6,
    kAcceptParams = 7
  };

  enum class CalibrationType : std::int8_t {
    kNone = 0,
    kUDS = 1,
    kDHU = 2
  };

  enum class CalibrationMode : std::int8_t {
    kNone = 0,
    kSVC = 1,
    kRLC = 2,
    kFLC = 3
  };

  // Calibration target select
  enum class EolCalibPattern : std::int8_t {
    kNone = 0,
    kPattern1 = 1,
    kPattern2 = 2
  };
  
  enum class WorkModeType : std::uint8_t {
    /// @brief work mode is none
    KNone = 0,
    /// @brief work mode is VPA
    KVPA = 1,
    /// @brief work mode is APA
    KAPA = 2,
    /// @brief work mode is RPA
    KRPA = 3,
    /// @brief work mode is RPA PARK OUT
    KRPA_PARK_OUT = 4
  };

  struct alignas(8) VisionSystem {
    std::int32_t parking_in_id;
    std::int32_t parking_out_id;
    /// @brief Driving or parking status
    VisionSystemStatus system_status;

    /// @brief calibration related data
    CalibrationStatus calibration_status;
    CalibrationType calib_type;
    CalibrationMode calib_mode;
    EolCalibPattern calib_pattern;

    bool enable_backstage_searching;
    bool enable_normal_searching;
    bool enable_PEB_PrkgPerception;
    MirrFoldStsTyp mirr_fold_sts_at_driver;
    MirrFoldStsTyp mirr_fold_sts_at_pass;
    LockIdType lock_id_type;
    VpaStateDetailType vpa_state_detail_type;
    WorkModeType work_mode_type;
    std::uint8_t padding[3];

    ApaStateInfo apa_states;
    UserSelectedParkingSlot usr_sel_ps;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit VisionSystemData(std::int64_t timestamp) noexcept;

  VisionSystemData() noexcept;
  ~VisionSystemData() noexcept final = default;
  VisionSystemData(VisionSystemData const&) = default;
  VisionSystemData& operator=(VisionSystemData const&) = default;
  VisionSystemData(VisionSystemData&&) = default;
  VisionSystemData& operator=(VisionSystemData&&) = default;

  /// @brief Get Vision System data
  ///
  /// @return vision_system_data_
  VisionSystem const& GetVisionSystem() const noexcept;

  /// @brief Mutable Vision System data
  ///
  /// @return vision_system_data_
  VisionSystem& MutableVisionSystem() noexcept;

  /// @brief Set Vision System data
  ///
  /// @param vision_system_data vision system data
  void SetVisionSystem(VisionSystem const& vision_system_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  VisionSystem vision_system_data_;
};

}  // namespace autodrive

#endif
