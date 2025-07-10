//
#ifndef AUTODRIVE_INTERFACE_PROCESSOR_INFO_DATA_H_
#define AUTODRIVE_INTERFACE_PROCESSOR_INFO_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

namespace autodrive {

class ProcessorInfoData final : public CommonMsgData {
 public:
  enum class ProcessorStatus : std::uint8_t {
    /// @brief Default status
    kCreated = 0,
    /// @brief After Init() donw
    kNormal = 1,
    /// @brief After recoverable error occurs
    kException = 2,
    /// @brief After unrecoverable error occurs
    kError = 3,
    /// @brief After Destroy() is called
    kDestroying = 4,
    /// @brief After Destroy() done
    kDestroyed = 5
  };

  enum class ErrorCode : std::uint8_t {
    kNone = 0,
    /// @brief Image data synchronized failed, image received but timestamp diff is too large
    kImageSyncFailed = 1,
    /// @brief Odo data not received
    kOdoCheckFailed = 2,
    /// @brief Read camera params files failed
    kCamFilesReadFailed = 3,
    /// @brief Timestamp diff between odo data and img data is too large
    kOdoTimestampFailed = 4,
    /// @brief StaticObstacle Check Failed
    kStaticObsCheckFailed = 5,
    /// @brief DynamicObstacle Check Failed
    kDynamicObsCheckFailed = 6,
    /// @brief UssObstacle Check Failed
    kUssObsCheckFailed = 7,
    /// @brief FrontObstacle Check Failed
    kFrontObsCheckFailed = 8,
    /// @brief ParkingSlots Check Failed
    kParkingSlotsCheckFailed = 9,
    /// @brief CylinderParsing Check Failed
    kCylinderParsingCheckFailed = 10,
    /// @brief IpmParsing Check Failed
    kIpmParsingCheckFailed = 11,
    /// @brief UssParkingSlots Check Failed
    kUssParkingSlotsCheckFailed = 12,
    /// @brief Image from one camera is required but not received
    kCameraOnlineCheckFaliled = 13
  };

  struct alignas(8) ProcessorInfo {
    /// @brief Status
    ProcessorStatus status;
    /// @brief Error code
    ErrorCode error_code;
    std::uint8_t padding[6];
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ProcessorInfoData(std::int64_t timestamp) noexcept;

  ProcessorInfoData() noexcept;
  ~ProcessorInfoData() noexcept final = default;
  ProcessorInfoData(ProcessorInfoData const&) = default;
  ProcessorInfoData& operator=(ProcessorInfoData const&) = default;
  ProcessorInfoData(ProcessorInfoData&&) = default;
  ProcessorInfoData& operator=(ProcessorInfoData&&) = default;

  /// @brief Get processor_info_data_
  ///
  /// @return processor_info_data_
  ProcessorInfo const& GetProcessorInfo() const noexcept;

  /// @brief Mutable processor_info_data_
  ///
  /// @return processor_info_data_
  ProcessorInfo& MutableProcessorInfo() noexcept;

  /// @brief Set processor_info_data_
  ///
  /// @return void
  void SetProcessorInfo(ProcessorInfo const& processor_info) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  ProcessorInfo processor_info_data_;
};

}  // namespace autodrive

#endif
