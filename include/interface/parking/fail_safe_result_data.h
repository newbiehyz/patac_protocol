//
#ifndef AUTODRIVE_INTERFACE_FAIL_SAFE_RESULT_DATA_H_
#define AUTODRIVE_INTERFACE_FAIL_SAFE_RESULT_DATA_H_

#include <foundation/type/common_msg_data.h>
#include "interface/parking/iqa_parsing_data.h"
#include "type/perception_base.h"

namespace autodrive {

class FailsafeResultData final : public CommonMsgData {
 public:
  /// @brief Max camera size
  static constexpr std::uint8_t kMaxCameraSize{5U};

  enum class SeverityLevel : std::uint8_t {
    kNone = 0,
    kMild = 1,
    kModerate = 2,
    kSevere = 3
  };

  enum class OutOfCalibTsrBs : std::uint8_t {
    kTsrOuutOfCalib = 0,
    kTsrOutOfCalibAeb = 1,
    kTsrOutOfCalibYaw = 2,
    kTsrOutOfCalibHorizon = 3
  };

  struct alignas(8) FailSafeLevel {
    SeverityLevel rain_severity_level;
    SeverityLevel full_blockage_severity_level;
    SeverityLevel blurred_image_severity_level;
    SeverityLevel splashes_severity_level;
    SeverityLevel low_sun_severity_level;
    SeverityLevel sun_ray_severity_level;
    bool out_of_focus_severity_level;
    bool out_of_calibration_severity_level;
    SeverityLevel partial_blockage_severity_level;
    SeverityLevel frozen_windshield_severity_level;
    SeverityLevel fog_severity_level;
    bool free_sight;
    OutOfCalibTsrBs out_of_calib_tsr_bs;
    std::uint8_t padding[3];
  };

  struct alignas(8) FailSafeResult {
    ImageId image_id;
    std::uint8_t padding[7];
    FailSafeLevel fail_safe_result;
    SceneParsing scene;
  };

  struct alignas(8) FailSafeResults{
    std::uint8_t num_fail_safe_result;
    std::uint8_t padding[7];
    std::array<FailSafeResult, kMaxCameraSize> fail_safe_result_data;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit FailsafeResultData(std::int64_t timestamp) noexcept;

  FailsafeResultData() noexcept;
  ~FailsafeResultData() noexcept final = default;
  FailsafeResultData(FailsafeResultData const&) = default;
  FailsafeResultData& operator=(FailsafeResultData const&) = default;
  FailsafeResultData(FailsafeResultData&&) = default;
  FailsafeResultData& operator=(FailsafeResultData&&) = default;

  /// @brief Get Fail Safe Level data
  ///
  /// @return fail_safe_result_data_
  FailSafeResults const& GetFailSafeResults() const noexcept;

  /// @brief Mutable Fail Safe Level data
  ///
  /// @return fail_safe_result_data_
  FailSafeResults& MutableFailSafeResults() noexcept;

  /// @brief Set Fail Safe Level data
  ///
  /// @param fail_safe_result_data fail safe result data
  void SetFailSafeResults(FailSafeResults const& fail_safe_result_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  FailSafeResults fail_safe_result_data_;
};

}  // namespace autodrive

#endif
