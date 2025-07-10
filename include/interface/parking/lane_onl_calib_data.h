//
#ifndef AUTODRIVE_INTERFACE_PARKING_LANE_ONL_CALIB_H_
#define AUTODRIVE_INTERFACE_PARKING_LANE_ONL_CALIB_H_

#include <foundation/type/common_msg_data.h>
#include <type/perception_base.h>

#include <array>

namespace autodrive {

class LaneOnlCalibData : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxParsingSize{2U};
  static constexpr std::uint32_t kImageWidth{1024};
  static constexpr std::uint32_t kImageHeight{576};
  static constexpr std::uint32_t kMaxCameraNumber{5U};

  struct alignas(8) LaneParsingImageData {
    std::uint8_t
        data[kImageWidth * kImageHeight];  // 0- obj_seg+Stop_line  1-Lane Attr
    std::uint32_t width;
    std::uint32_t height;
    std::uint8_t channel;
    std::uint8_t padding[7];
  };

  struct alignas(8) SingleCamLane {
    std::int32_t camera_id;
    std::uint8_t num_lane_parsing_image;
    std::uint8_t padding[3];
    std::array<LaneParsingImageData, kMaxParsingSize> lane_parsing_image;
    Rect parsing_roi;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit LaneOnlCalibData(std::int64_t timestamp) noexcept;

  LaneOnlCalibData() noexcept;
  ~LaneOnlCalibData() noexcept final = default;
  LaneOnlCalibData(LaneOnlCalibData const &) = default;
  LaneOnlCalibData &operator=(LaneOnlCalibData const &) = default;
  LaneOnlCalibData(LaneOnlCalibData &&) = default;
  LaneOnlCalibData &operator=(LaneOnlCalibData &&) = default;

  /// @brief Get camera_lane_
  ///
  /// @return camera_lane_
  std::array<SingleCamLane, kMaxCameraNumber> const &GetCameraLane()
      const noexcept;

  /// @brief Mutable camera_lane_
  ///
  /// @return camera_lane_
  std::array<SingleCamLane, kMaxCameraNumber> &MutableCameraLane() noexcept;

  /// @brief Set camera_lane_
  ///
  /// @return void
  void SetCameraLane(
      std::array<SingleCamLane, kMaxCameraNumber> const &camera_lane) noexcept;

  /// @brief get num_camera_lane_
  ///
  /// return uint8_t
  void SetCameraNum(const std::uint8_t &num_camera_lane) noexcept;

  /// @brief get num_camera_lane_
  ///
  /// return uint8_t
  uint8_t const &GetCameraNum() const noexcept;

  /// @brief get num_camera_lane_
  ///
  /// return uint8_t
  uint8_t &MutableCameraNum() noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  uint8_t num_camera_lane_;
  std::array<SingleCamLane, kMaxCameraNumber> camera_lane_;
};

}  // namespace autodrive

#endif
