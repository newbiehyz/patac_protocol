//
#ifndef AUTODRIVE_MODEL_CALIBRATION_RESULT_DATA_H_
#define AUTODRIVE_MODEL_CALIBRATION_RESULT_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "model/camera_extrinsics_data.h"
namespace autodrive {

// Calibration result data
class CalibrationResultData final : public CommonMsgData {
 public:
  static constexpr std::uint8_t kMaxCameraSize{5U};

  enum class CalibStatus : std::int8_t {
    CALIB_STATE_RUN = 1,                   // 进行中
    CALIB_STATE_SUCCESS = 2,               // 标定成功
    CALIB_STATE_MODE_ERRROR = 3,           // 标定模式输入错误
    CALIB_STATE_PATTERN_ERROR = 4,         // 靶标设置错误
    CALIB_STATE_INTERNAL_PARAM_ERROR = 5,  // 内参校验失败
    CALIB_STATE_IMG_ERRROR = 6,            // 图像错误
    CALIB_STATE_FIND_CORNERS_ERROR = 7,    // 角点提取失败
    CALIB_STATE_REPROJECTION_ERROR = 8,    // VCS重投影误差过大
    CALIB_STATE_PITCH_OUTOF_RANGE = 9,     // 标定pitch超限
    CALIB_STATE_ROLL_OUTOF_RANGE = 10,     // 标定roll超限
    CALIB_STATE_YAW_OUTOF_RANGE = 11,       // 标定yaw超限
    CALIB_STATE_TIMEOUT_ERROR = 12,         //超时
    CALIB_STATE_RESULT_SAVE_ERROR = 13,      //保存失败
    CALIB_STATE_ALGORITHM_CANCEL = 14      //算法已取消
  };

  struct alignas(8) CalibrationResult {
    float progress_rate;    // 道路标定进度
    CameraID cam_id;
    CalibStatus calib_status;
    std::uint8_t padding[2];
    CameraExtrinsicsData::Extrinsic extrinsic;
  };

  struct alignas(8) CalibrationResults {
    /// The actual number of cameras used by the project
    std::uint8_t real_camera_amount;
    std::uint8_t num_results;
    std::uint8_t padding[6];
    std::array<CalibrationResult, kMaxCameraSize> results;
  };

  CalibrationResultData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit CalibrationResultData(std::int64_t timestamp) noexcept;
  ~CalibrationResultData() noexcept = default;
  CalibrationResultData(CalibrationResultData const&) = default;
  CalibrationResultData& operator=(CalibrationResultData const&) = default;
  CalibrationResultData(CalibrationResultData&&) = default;
  CalibrationResultData& operator=(CalibrationResultData&&) = default;

  /// @brief Get function mode
  ///
  /// @return function_mode_
  CalibrationResults const& GetCalibration() const noexcept;

  /// @brief Get mutable function mode
  ///
  /// @return function_mode_
  CalibrationResults& MutableCalibration() noexcept;

  /// @brief Set function mode
  ///
  /// @return function_mode_
  void SetCalibration(const CalibrationResults& calibration) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  CalibrationResults calib_result_;
};

}  // namespace autodrive

#endif