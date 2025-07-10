//
#ifndef AUTODRIVE_INTERFACE_ONL_CALIBRATION_DATA_H_
#define AUTODRIVE_INTERFACE_ONL_CALIBRATION_DATA_H_

#include <foundation/type/common_msg_data.h>
#include "model/camera_extrinsics_data.h"

namespace autodrive{

class OnlCalibrationData final : public CommonMsgData {
public:
  static constexpr std::uint8_t kMaxCameraSize{5U};

  enum class CalibFinalState : std::uint8_t {
    CALIB_NONE = 0,
    CALIB_STATE_SUCESS = 1
  };

  enum class CalibStatus : std::uint64_t {
    CAM_STATUS_DISABLE = 0,
    CAM_STATUS_ENABLE = 1,
    CAM_STATUS_FINISH = 2,
    CAM_STATUS_PITCH_OK = 4,
    CAM_STATUS_YAW_OK = 8,
    CAM_STATUS_ROLL_OK = 16,
    CAM_STATUS_NOT_SURPPORT = 32,
    CAM_STATUS_PITCH_SCENE_IMPROPER = 64,
    CAM_STATUS_YAW_SCENE_IMPROPER = 128,
    CAM_STATUS_ROLL_SCENE_IMPROPER = 256,
    CAM_STATUS_PITCH_NEED_TO_RECALIB = 512,
    CAM_STATUS_YAW_NEED_TO_RECALIB = 1024,
    CAM_STATUS_ROLL_NEED_TO_RECALIB = 2048,
    CAM_STATUS_LOOSE_PITCH = 4096,
    CAM_STATUS_LOOSE_YAW = 8192,
    CAM_STATUS_LOOSE_ROLL = 16384,
  };

  struct OnlCalibrationResult{
    CameraID cam_id;
    CameraExtrinsicsData::Extrinsic extrinsic;
    CalibStatus cam_status;
  };

  struct OnlCalibrationResults{
    std::uint32_t num_results;
    std::array<OnlCalibrationResult, kMaxCameraSize> results;
  };

  OnlCalibrationData() noexcept;
  ~OnlCalibrationData() noexcept final = default;
  OnlCalibrationData(OnlCalibrationData const&) = default;
  OnlCalibrationData& operator=(OnlCalibrationData const&) = default;
  OnlCalibrationData(OnlCalibrationData&&) = default;
  OnlCalibrationData& operator=(OnlCalibrationData&&) = default;

  CalibFinalState const& GetCalibFinalState() const noexcept;
  void SetCalibFinalState(const CalibFinalState &final_state) noexcept;
  CalibFinalState& MutableCalibFinalState() noexcept;

  OnlCalibrationResults const& GetOnlCalibrationResults() const noexcept;
  void SetOnlCalibrationResults(const OnlCalibrationResults &results) noexcept;
  OnlCalibrationResults& MutableOnlCalibrationResults() noexcept;
 
   /// @brief Clear data
  void Clear() noexcept;

private:
  CalibFinalState final_state_;
  OnlCalibrationResults calib_result_;
};

}

#endif // AUTODRIVE_INTERFACE_ONL_CALIBRATION_DATA_H_