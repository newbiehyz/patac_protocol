//
#ifndef AUTODRIVE_INTERFACE_LANE_MULTITASK_H_
#define AUTODRIVE_INTERFACE_LANE_MULTITASK_H_

#include <array>

#include "road_info_data.h"
#include "type/parsing_base.h"

namespace autodrive {

class LaneMultitaskData : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxParsingSize{2U};
  static constexpr std::uint32_t kMaxTransitionPointSize{8U};
  static constexpr std::uint32_t kImageWidth{1024};
  static constexpr std::uint32_t kImageHeight{576};
  static constexpr std::uint32_t kLaneMarkChannel{8};
  static constexpr std::uint32_t kLaneMarkWidth{256};
  static constexpr std::uint32_t kLaneMarkHeight{144};
  static constexpr std::uint32_t kMaxBasicPtsNum{128};
  static constexpr std::uint32_t kMaxExtraPtsNum{128};
  static constexpr std::uint32_t kMaxLaneCountoursNum{16};

  struct alignas(8) LaneParsingImageData {
    std::uint8_t
        data[kImageWidth * kImageHeight];  // 0- obj_seg+Stop_line  1-Lane Attr
    std::uint32_t width;
    std::uint32_t height;
    std::uint8_t channel;
    std::uint8_t padding[7];
  };

  struct alignas(8) LaneMarkParsingData {
    float data[kLaneMarkHeight * kLaneMarkWidth *
               kLaneMarkChannel];  // h,w,c;0-3,laneEdge, 4- edge;
    std::uint32_t width;
    std::uint32_t height;
    std::uint8_t channel;
    std::uint8_t padding[7];
  };

  struct alignas(8) LaneTransitionPoint {
    std::array<RoadInfoData::TransitionPoints, kMaxTransitionPointSize>
        transition_points;
    uint8_t num_transition_point;
    std::uint8_t padding[7];
  };

  struct alignas(8) BasicInfoPts {
    Vector2f pt;         // in used coordinate, local / lane .. etc
    Vector2f pt_rle;     // in parsing image
    Vector2f pt_oriimg;  // in raw image
    Vector2f pt_img;     // in undistort image
    uint64_t label;      // parsing label
    int8_t conf;
    std::uint8_t padding[7];
  };

  struct alignas(8) ExtraInfoPts {
    std::int32_t xstart;
    std::int32_t xend;
    std::int32_t y_parsing;
    std::int32_t gray;
    Vector2f pt_rle_c;
    Vector2f width_info;
    bool flag_dash_endpt;
    bool horizontal;
    std::uint8_t padding[6];
  };

  struct alignas(8) LaneContour {
    std::uint8_t num_basic_pts;
    std::uint8_t num_extra_pts;
    std::uint8_t padding[6];
    std::array<BasicInfoPts, kMaxBasicPtsNum> basic_pts;
    std::array<ExtraInfoPts, kMaxExtraPtsNum> extra_pts;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit LaneMultitaskData(std::int64_t timestamp) noexcept;

  LaneMultitaskData() noexcept;
  ~LaneMultitaskData() noexcept final = default;
  LaneMultitaskData(LaneMultitaskData const&) = default;
  LaneMultitaskData& operator=(LaneMultitaskData const&) = default;
  LaneMultitaskData(LaneMultitaskData&&) = default;
  LaneMultitaskData& operator=(LaneMultitaskData&&) = default;

  /// @brief Get lane_parsing_image
  ///
  /// @return lane_parsing_image_
  std::array<LaneParsingImageData, kMaxParsingSize> const& GetFeatureMap()
      const noexcept;

  /// @brief Mutable lane_parsing_image
  ///
  /// @return lane_parsing_image_
  std::array<LaneParsingImageData, kMaxParsingSize>&
  MutableFeatureMap() noexcept;

  // @brief Set lane_parsing_image_
  ///
  /// @param lane_parsing_image
  void SetFeatureMap(std::array<LaneParsingImageData, kMaxParsingSize> const&
                         lane_parsing_image) noexcept;

  /// @brief Get rle_lane_parsing_image_
  ///
  /// @return rle_lane_parsing_image_
  std::array<RleCompressedData, kMaxParsingSize> const& GetRleFeatureMap()
      const noexcept;

  /// @brief Mutable rle_lane_parsing_image_
  ///
  /// @return rle_lane_parsing_image_
  std::array<RleCompressedData, kMaxParsingSize>&
      MutableRleFeatureMap() noexcept;

  /// @brief Get lane_transition_point_
  ///
  /// @return lane_transition_point_
  LaneTransitionPoint const& GetTransitionPoint() const noexcept;

  /// @brief Mutable lane_transition_point_
  ///
  /// @return lane_transition_point_
  LaneTransitionPoint& MutableTransitionPoint() noexcept;

  // @brief Set lane_transition_point_
  ///
  /// @param lane_transition_point
  void SetTransitionPoints(
      LaneTransitionPoint const& lane_transition_point) noexcept;

  /// @brief Get num_lane_parsing_image_
  ///
  /// @return num_lane_parsing_image_
  std::uint8_t GetLaneParsingImageNum() const noexcept;

  // @brief Set num_lane_parsing_image_
  ///
  /// @param num_lane_parsing_image_
  void SetLaneParsingImageNum(std::uint8_t num) noexcept;

  std::array<LaneContour, kMaxLaneCountoursNum> const& GetLaneContours() noexcept;

  std::uint8_t GetLaneContourNum() const noexcept;

  void SetLaneContourNum(std::uint8_t num) noexcept;

  std::array<LaneContour, kMaxLaneCountoursNum> &MutableLaneContours() noexcept;

  /// @brief Get camera_id_
  ///
  /// @return camera_id_
  std::int32_t GetCameraId() const noexcept;

  // @brief Set camera_id_
  ///
  /// @param camera_id_
  void SetCameraId(std::int32_t camera_id) noexcept;

  /// @brief Get roi
  ///
  /// @return roi
  Rect const& GetRoi() const noexcept;

  /// @brief Mutable roi
  ///
  /// @return roi
  Rect& MutableRoi() noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  std::int32_t camera_id_;
  LaneTransitionPoint lane_transition_point_;
  std::uint8_t num_lane_parsing_image_;
  std::array<LaneParsingImageData, kMaxParsingSize> lane_parsing_image_;
  std::array<RleCompressedData, kMaxParsingSize> rle_lane_parsing_image_;
  std::uint8_t num_lane_contour_;
  std::array<LaneContour, kMaxLaneCountoursNum> lane_contours_;
  Rect roi_;
};

}  // namespace autodrive
#endif
