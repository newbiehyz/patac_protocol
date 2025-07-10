//
#ifndef AUTODRIVE_INTERFACE_PARSING_DATA_H_
#define AUTODRIVE_INTERFACE_PARSING_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/parsing_base.h"
#include "type/perception_base.h"

namespace autodrive {

class ParsingData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxRoiMapSize{4U};
  static constexpr std::uint32_t kMaxRotationSize{3U};
  static constexpr std::uint32_t kMaxTranslationSize{3U};
  static constexpr std::uint32_t kMaxDistortParamSize{4U};
  static constexpr std::uint32_t kMaxChannelLabelsSize{30U};
  static constexpr std::uint32_t kMaxChannelColorsSize{30U};
  static constexpr std::uint32_t kModelNameSize{50U};
  static constexpr std::uint32_t kMaxFreespaceChannelsSize{11U};
  static constexpr std::uint32_t kMaxBoundspaceChannelsSize{11U};
  static constexpr std::uint32_t kMaxModelInfoSize{4U};
  static constexpr std::uint32_t kMaxCategoriesSize{4U};
  static constexpr std::uint32_t kImageWidth{448U};
  static constexpr std::uint32_t kImageHeight{448U};

  enum class CompressionFormat : std::uint8_t {
    kOriginalImage = 0,
    /// @brief Width and height sample to 1/2
    kOriginalImageHalf = 1,
    /// @brief Width and height sample to 1/4
    kOriginalImageQuarter = 2,
    kCompressionJPEG = 3,
    kCompressionH265I = 4,
    kCompressionH265P = 5
  };

  enum class ImageFormat : std::uint8_t {
    kGray = 0,
    kYv12 = 1,
    kJpeg = 2,
    kPng = 3,
    kCr12 = 4,
    kBad = 5,
    kNv12 = 6,
    kNv21 = 7,
    kTimeout = 8,
    kBypassOnly = 9,
    kYuyv = 10,
    kUyvy = 11,
    kRaw12Planar = 12,
    kRaw12PlanarJpeg = 13,
    kRaw12PlanarJpegLossless = 14,
    kRaw12 = 15,
    kRaw12Jpeg = 16,
    kRaw12JpegLossless = 17,
    kRaw16 = 18,
    kRaw16Jpeg = 19,
    kRaw16JpegLossless = 20,
    kRaw20 = 21,
    kRaw20Jpeg = 22,
    kRaw20JpegLossless = 23,
    kRaw20Planar = 24,
    kRaw20PlanarJpeg = 25,
    kRaw20PlanarJpegLossless = 26,
    kRaw16WithFront = 27,
    kRaw16WithRear = 28,
    kRaw16WithFrontRear = 29,
    kRaw16PlanarJpegLossless = 30,
    kRaw14PlanarJpegLossless = 31,
    kBitstream = 32,
    kBitstreamH264 = 33
  };

  enum class ParsingFormat : std::uint8_t {
    kParsingFormatRaw = 0,
    kParsingFormatLabel = 1,
    kParsingFormatLabelRle = 2
  };

  enum class IPMSelect : std::uint8_t {
    kNinetyDistant = 0,
    kFortyDistant = 1,
    kFortyHalfWidth = 2,
    kTwentyDistant = 3
  };

  enum class PerceptionStatus : std::uint8_t {
    kNoPerception = 0,
    kWithPerception = 1,
  };

  enum class ImageStatus : std::uint8_t {
    kImageStatusNormal = 0,
    kImageStatusNoInterrupt = 1,
  };

  struct alignas(8) Image {
    std::int32_t width;
    std::int32_t height;
    std::int32_t channel;
    std::int32_t send_mod;
    std::int64_t time_stamp;

    /// @brief nv12/yv12/raw12...
    std::int32_t format;
    std::int32_t index;
    std::int32_t count;
    std::int32_t depth;
    std::int32_t align;
    /// @brief Corresponding to the type of frame_id in VIO image data structure
    std::uint32_t image_counter;
    std::int32_t perception_status;
    std::uint8_t num_roi_map;
    std::uint8_t padding[3];
    /// @brief Bypass's ts relative to net meta's, valid in bypass scenario
    std::int64_t bypass_time_diff;

    /// @brief jpeg/original/original half/original quarter...
    std::int32_t compression_format;
    std::int32_t status;
    std::array<RoiMap, kMaxRoiMapSize> roi_map;

    std::int32_t perception_mode;
    std::int32_t postprocessor_status;
    std::int32_t frame_id;
    std::int32_t step;
    std::int64_t time_stamp_us;
    std::uint64_t hw_time_stamp;
    /**
     * @brief Timestamp difference between consecutive struct alignas(8)s
     *  received by mcu in nanoseconds; value of this filed is
     *  desired to be(or approximate) 1E9, if the difference between
     *  this value and 1E9 is is too large, time of mcu is unstable
     */
    std::int64_t mcu_time_diff;
    /**
     * @brief Timestamp difference between mcu and J3pilot(or other) board
     *  in nanoseconds
     */
    std::int64_t mcu_board_timediff;
  };

  enum class CameraType : std::uint8_t { kPinHole = 0, kFishEye = 1 };

  struct alignas(8) CameraValidHeight {
    std::int32_t left_y;
    std::int32_t right_y;
  };

  struct alignas(8) VCSParam {
    std::uint8_t num_rotation;
    std::uint8_t num_translation;
    std::uint8_t padding[6];
    /// @brief Rotation relative to x, y, z axis
    std::array<float, kMaxRotationSize> rotation;
    std::array<float, kMaxTranslationSize> translation;
  };

  enum class CameraVersion : std::uint8_t {
    kCV = 0,
    kVCS = 1,
  };

  enum class ParsingCalibrationStatus : std::uint8_t {
    kDefault = 0,
    kIntrinsicDone = 1,
    kExtrinsicDone = 2
  };

  enum class CalibrationType : std::uint8_t {
    kUnknown = 0,
    kEOL = 1,
    kAfterSale = 2,
    kBackground = 3
  };

  struct alignas(8) DistortParam {
    std::uint8_t num_param;
    std::uint8_t padding[7];
    std::array<float, kMaxDistortParamSize> param;
  };

  struct alignas(8) CameraParam {
    float focal_u;
    float focal_v;
    float center_u;
    float center_v;
    float camera_x;
    float camera_y;
    float camera_z;
    float pitch;
    float yaw;
    float roll;
    std::int32_t type;
    float fov;
    /// @brief Version
    // 0: old camera, front:z, right:x, down:y
    // 1: new camera, front:x, left: y, up: z
    std::int32_t version;
    ParsingCalibrationStatus cali_status;
    std::uint8_t padding_1[3];
    CameraValidHeight valid_height;
    DistortParam distort;
    VCSParam vcs;
    CameraMatrix mat;
    std::int64_t time_stamp;
    std::int32_t cam_id;
    std::int32_t frame_id;
    std::int32_t image_height;
    std::int32_t image_width;
    /// @brief Base calib done
    // 0 No basic calibration; 1 EOL calibration completed; 2 After-sales
    // calibration completed
    std::int32_t base_calib_done;
    CalibrationType calib_src;
    std::uint8_t padding_2[3];
    /// @brief Calib done ts
    // Calibrate the order placement time, the number of milliseconds elapsed
    // since UTC January 1, 1970
    std::int64_t calib_done_ts;
  };

  struct alignas(8) ParsingColor {
    std::uint32_t b;
    std::uint32_t g;
    std::uint32_t r;
    std::uint8_t padding[4];
  };

  struct alignas(8) RuntimeModelInfo {
    std::int32_t model_id;
    std::array<char, kModelNameSize> model_name;
    std::uint16_t num_model_name;
    std::uint8_t padding[4];
  };

  enum class ParsingType : std::uint8_t {
    kDefault = 0,
    kLane = 1,
    kDepth = 2,
    kIpm = 3,
    kBev = 4,
    kIqa = 5
  };

  struct alignas(8) ParsingSetting {
    PerceptionHeader header;
    std::int32_t parsing_type;
    /// @brief ReservedFields
    std::int32_t parsing_channels;

    std::uint8_t num_channel_labels;
    std::uint8_t num_channel_colors;
    std::uint8_t num_freespace_channels;
    std::uint8_t num_boundspace_channels;
    std::uint8_t num_model_info;
    std::uint8_t num_categories;
    std::uint8_t padding_1[2];
    /// @brief ReservedFields
    std::array<std::uint64_t, kMaxChannelLabelsSize> channel_labels;
    /// @brief ReservedFields
    std::array<ParsingColor, kMaxChannelColorsSize> channel_colors;
    std::array<std::int32_t, kMaxFreespaceChannelsSize> freespace_channels;
    /// @brief ReservedFields
    std::array<std::int32_t, kMaxBoundspaceChannelsSize> boundspace_channels;
    /// @brief ReservedFields
    Rect parsing_roi;
    float parsing_scale;
    std::uint8_t padding_2[4];
    /// @brief ReservedFields
    std::array<RuntimeModelInfo, kMaxModelInfoSize> model_info;
    /// @brief ReservedFields
    std::array<Category, kMaxCategoriesSize> categories;
  };

  struct alignas(8) DepthSetting {
    /// @brief ReservedFields
    float scale_number;
    /// @brief ReservedFields
    float shift_scale;
    /// @brief ReservedFields
    std::uint32_t mapping_type;
    /// @brief ReservedFields
    std::uint32_t mapping_shift;
    /// @brief ReservedFields
    float confidence_scale;
    /// @brief ReservedFields
    float max_depth;
    /// @brief ReservedFields
    float min_depth;
    /// @brief ReservedFields
    float max_confidence;
    /// @brief ReservedFields
    float min_confidence;
    std::uint8_t padding[4];
    /// @brief ReservedFields
    CameraParam camera_param;
  };

  struct alignas(8) ParsingImageData {
    std::uint8_t data[kImageWidth * kImageHeight];
    std::uint8_t width;
    std::uint8_t height;
    std::uint8_t channel;
    std::uint8_t padding[5];
  };

  struct alignas(8) Parsing {
    /// @brief Parsing point free or occupied(0-occupied; 1-free)
    ParsingImageData free_space;
    /// @brief Parsing point label
    ParsingImageData feature_map;
    /// @brief Rle parsing result
    RleCompressedData rle_parsing_compressed;
    /// @brief Parsing scale comparing to camera img width
    float scale;
    std::uint8_t padding[4];
    /// @brief Valid rit on parsing img
    Rect roi_output;
    /// @brief ReservedFields
    Image image;
    /// @brief ReservedFields
    ParsingSetting parsing_setting;
    /// @brief ReservedFields
    DepthSetting depth_setting;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ParsingData(std::int64_t timestamp) noexcept;

  ParsingData() noexcept;
  ~ParsingData() noexcept final = default;
  ParsingData(ParsingData const&) = default;
  ParsingData& operator=(ParsingData const&) = default;
  ParsingData(ParsingData&&) = default;
  ParsingData& operator=(ParsingData&&) = default;

  /// @brief Get Parsing data
  ///
  /// @return parsing_data_
  Parsing const& GetParsing() const noexcept;

  /// @brief Mutable Parsing data
  ///
  /// @return parsing_data_
  Parsing& MutableParsing() noexcept;

  /// @brief Set Parsing data
  ///
  /// @param parsing_data parsing_data
  void SetParsing(Parsing const& parsing_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  Parsing parsing_data_;
};

}  // namespace autodrive

#endif
