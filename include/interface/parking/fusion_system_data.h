//
#ifndef AUTODRIVE_INTERFACE_FUSION_SYSTEM_DATA_H_
#define AUTODRIVE_INTERFACE_FUSION_SYSTEM_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/system_type_base.h"

namespace autodrive {

class FusionSystemData final : public CommonMsgData {
 public:
  enum class ParkingOutDir : std::uint8_t {
    /// @brief Parking out unknow
    KOutUnknow = 0,
    /// @brief vertical front left
    KVerFrontLeft = 1,
    /// @brief vertical front center
    KVerFrontCenter = 2,
    /// @brief vertical front right
    KVerFrontRight = 3,
    /// @brief vertical rear left
    KVerRearLeft = 4,
    /// @brief vertical rear center
    KVerRearCenter = 5,
    /// @brief vertical rear right
    KVerRearRight = 6,
    /// @brief horizontal front left
    KHoriFrontLeft = 7,
    /// @brief horizontal front right
    KHoriFrontRight = 8
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

  enum class PerceptionCtrlStatus : std::uint8_t{
    /// @brief PerceptionCtrlStatus is unknown
    kUnknown = 0,
    /// @brief PerceptionCtrlStatus switch to parking
    kSwitchParking = 1,
    /// @brief PerceptionCtrlStatus switch to ADAS
    kSwitchADAS = 2,
    /// @brief PerceptionCtrlStatus is parking
    kParking = 3,
    /// @brief PerceptionCtrlStatus is adas
    kADAS = 4
  };

  /// @brief Draft
  struct alignas(8) FusionSystem {
    /// @brief Parking spot ID
    std::int32_t lock_id;
    /// @brief Corresponding lock parking spot type
    LockIdType lock_id_type;
    bool enable_backstage_searching;
    bool enable_normal_searching;
    bool enable_PEB_PrkgPerception;
    ParkingOutDir parking_out_dir;
    WorkModeType work_mode_type;
    MirrFoldStsTyp mirr_fold_sts_at_driver;
    MirrFoldStsTyp mirr_fold_sts_at_pass;
    VpaStateDetailType vpa_state_detail_type;
    PerceptionCtrlStatus perception_ctrl_status;
    std::uint8_t padding[2];

    UserSelectedParkingSlot usr_sel_ps;
    ApaStateInfo apa_states;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit FusionSystemData(std::int64_t timestamp) noexcept;

  FusionSystemData() noexcept;
  ~FusionSystemData() noexcept final = default;
  FusionSystemData(FusionSystemData const&) = default;
  FusionSystemData& operator=(FusionSystemData const&) = default;
  FusionSystemData(FusionSystemData&&) = default;
  FusionSystemData& operator=(FusionSystemData&&) = default;

  /// @brief Get Fusion System data
  ///
  /// @return fusion_system_data_
  FusionSystem const& GetFusionSystem() const noexcept;

  /// @brief Mutable Fusion System data
  ///
  /// @return fusion_system_data_
  FusionSystem& MutableFusionSystem() noexcept;

  /// @brief Set Fusion System data
  ///
  /// @param fusion_system_data fusion system data
  void SetFusionSystem(FusionSystem const& fusion_system_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  FusionSystem fusion_system_data_;
};

}  // namespace autodrive

#endif
