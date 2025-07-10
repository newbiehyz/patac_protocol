//
#include "model/calibration_result_data.h"

#include <cstring>

namespace autodrive {

CalibrationResultData::CalibrationResultData() noexcept :
    CommonMsgData{}, calib_result_{0U} {
}

CalibrationResultData::CalibrationResultData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp}, calib_result_{0U} {
}

CalibrationResultData::CalibrationResults const& CalibrationResultData::GetCalibration()
    const noexcept {
  return calib_result_;
}

CalibrationResultData::CalibrationResults& CalibrationResultData::MutableCalibration()
    noexcept {
  return calib_result_;
}

void CalibrationResultData::SetCalibration(const CalibrationResults& calibration) noexcept {
  calib_result_ = calibration;
}

void CalibrationResultData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&calib_result_, 0, sizeof(calib_result_));
}

}