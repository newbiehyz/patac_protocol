//
#include "interface/parking/onl_calibration_data.h"

#include <cstring>

namespace autodrive {

OnlCalibrationData::OnlCalibrationData() noexcept : CommonMsgData{}, calib_result_{} {}

OnlCalibrationData::CalibFinalState const& OnlCalibrationData::GetCalibFinalState() const noexcept {
  return final_state_;
}

OnlCalibrationData::OnlCalibrationResults const& OnlCalibrationData::GetOnlCalibrationResults() const noexcept {
  return calib_result_;
}

OnlCalibrationData::CalibFinalState& OnlCalibrationData::MutableCalibFinalState() noexcept {
  return final_state_;
}

OnlCalibrationData::OnlCalibrationResults& OnlCalibrationData::MutableOnlCalibrationResults() noexcept {
  return calib_result_;
}

void OnlCalibrationData::SetCalibFinalState(CalibFinalState const& final_state) noexcept {
  final_state_ = final_state;
}

void OnlCalibrationData::SetOnlCalibrationResults(OnlCalibrationResults const& calib_result) noexcept {
  calib_result_ = calib_result;
}

void OnlCalibrationData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&final_state_, 0, sizeof(final_state_));
  memset(&calib_result_, 0, sizeof(calib_result_));
}

}  // namespace autodrive
