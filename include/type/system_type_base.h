//
#ifndef AUTODRIVE_SYSTEM_TYPE_BASE_H_
#define AUTODRIVE_SYSTEM_TYPE_BASE_H_

#include <cstdint>

#include "type/parking_slot.h"

namespace autodrive {

enum class ApaFunctionStage : std::uint8_t {
  kFunctionNone = 0,
  /// @brief Searching
  kApaSearching = 1,
  /// @brief Before parking, the HMI has a front display
  kApaPreParking = 2,
  /// @brief Parking
  kApaParking = 3,
  /// @brief Parking complete
  kApaComplete = 4,
  /// @brief Parking abort
  kApaAbort = 5,
  /// @brief Parking suspension
  kApaSuspend = 6,
  /// @brief Fault
  kApaFailed = 7,
  kApaOther = 8,
};

enum class ApaStateDetailType : std::uint8_t {
  kStateNone = 0,
  /// @brief Power-on self-test or power-off state
  kOff = 1,
  /// @brief Power-On Self-Test Passed
  kStandby = 2,
  /// @brief Fault
  kFailed = 3,
  /// @brief Function Exit
  kAbort = 4,
  /// @brief Background search in progress, scene judged as parking scene
  kSearchingParkingIn = 5,
  /// @brief Background search in progress, scene judged as unparking scene
  kSearchingParkingOut = 6,
  /// @brief Before parking, the HMI has a foreground display
  kPreParkingIn = 7,
  /// @brief Before unparking, the HMI has a foreground display
  kPreParkingOut = 8,
  /// @brief Parking in progress
  kParkingIn = 9,
  /// @brief Unparking in progress
  kParkingOut = 10,
  /// @brief APA function paused and cannot be resumed
  kSuspendNotContinue = 11,
  /// @brief APA function paused and can be resumed
  kSuspendContinue = 12,
  /// @brief Parking complete
  kComplete = 13,
  kPreParkingSelect = 14
};

enum class VpaStateDetailType : std::uint8_t {
  kStateNone = 0,
  // request map weak active signal util user click or map stop trigger
  kRequestMapWeakActive = 1,
  kInitMap = 2,              // init map funciton
  kBuildMapClickable = 3,   // request build map
  kMapBuildRunning = 4,     // after user start map build
  kMapBuildComplete = 5,    // map build complete;
  kMapBuildExit = 6,        // map build failed with reason
  kMapBuildOptimizing = 7,  // map save and optimizing(用户倒车入库)
  kMapBuildApa = 8,         // APA 泊车入库
  kMapBuildDrivePark = 9,  // 驾驶员倒车入库
  // request vap active signal util user click or map stop trigger
  kRequestVpaWeakActive = 10,
  kInitVpa = 11,             // init vpa function
  kStartVpaClickable = 12,  // request start vpa
  kVpaRunning = 13,          // after user start vpa
  kVpaRandomApa = 14,       // 任意车位泊车
  kVpaFixApa = 15,          // 记忆车位泊车
  kVpaComplete = 16,         // vpa complete;
  kVpaExit = 17,             // vpa failed with reason
  kVpaPause = 18,            // 各种原因的暂停
  kVpaContinue = 19,         // 修复暂停原因后的可继续状态
  kVpaEpsInterrupt = 20,    // EPS打断
  kVpaEpsContinue = 21,     // EPS可继续
  kDiag = 22,                 // system diag with diag type
  kPureApa = 23,  // 纯apa状态
  kPureRpa = 24,  // 纯rpa状态
  kMapPause = 25,    // MAP apa 的暂停
  kMapContinue = 26  // MAP APA的可继续状态
};

struct alignas(8) ApaStateInfo {
  ApaStateDetailType state_type;
  ApaFunctionStage func_stage;
  std::uint8_t padding[6];
};

struct alignas(8) UserSelectedParkingSlot {
  bool user_sel_parking_slot_received;
  std::uint8_t padding[7];
  ParkingSlotInfo usersel_parking_slot;
};

enum class MirrFoldStsTyp : std::uint8_t {
  MirrFoldStsTyp_MirrFoldPosnUndefd = 0,
  MirrFoldStsTyp_MirrNotFoldPosn = 1,
  MirrFoldStsTyp_MirrFoldPosn = 2,
  MirrFoldStsTyp_MirrMovgToNotFold = 3,
  MirrFoldStsTyp_MirrMovgToFold = 4
};

enum class LockIdType : std::uint8_t {
    /// @brief Lock Id Type unknow
    KNone = 0,
    /// @brief Lock Id of Parking In
    KParkingIn = 1,  // parking in type
    /// @brief Lock Id of parking out vision
    KParkingOutVision = 2,  // parking out vision
    /// @brief Lock Id of parking out uss
    KParkingOutUss = 3, // parking out uss
    KParkingRSPA = 4 // rspa
  };

}  // namespace autodrive

#endif
