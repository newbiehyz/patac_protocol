//
#ifndef AUTODRIVE_INTERFACE_IPM_PARSING_DATA_H_
#define AUTODRIVE_INTERFACE_IPM_PARSING_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/parsing_base.h"

namespace autodrive {

class IpmParsingData final : public CommonMsgData {
 public:
  enum class IpmParsingLabel : uint8_t {
    kRoad = 0,
    kTrafficLine = 1,
    kParkingLine = 2,
    kParkingSpace = 3,
    kTrafficArrow = 4,
    kGuideLine = 5,
    kCrosswalkLine = 6,
    kNoParkingSignLine = 7,
    kStopLine = 8,
    kSpeedBump = 9,
    kSignLine = 10,
    kParkingLockOpen = 11,
    kParkingLockClosed = 12,
    kTrafficCone = 13,
    kParkingRod = 14,
    kCurb = 15,
    kColumn = 16,
    kImmovableObstacle = 17,
    kMovableObstacle = 18,
    kOther = 19,
    kSidewalk = 20,
    kPaintedWallRoot = 21
  };

  enum class FreeSpaceLabel : uint8_t {
    kBackGround = 0,
    kImmovableObstacle = 1,
    kMovableObstacle = 2,
    kParkingRod = 3,
  };

  struct alignas(8) IpmParsing {
    IpmParsingImageData ground_pt;
    IpmParsingImageData feature_map;

    /// @brief Rle compressed parsing results
    RleCompressedData rle_ground_pt;
    RleCompressedData rle_feature_map;

    /// @brief header of connect region
    ConnectRegion* region_ground_pt{nullptr};
    ConnectRegion* region_feature_map{nullptr};

    /// @brief Parsing scale comparing to camera img width
    float scale;
    std::uint8_t padding[4];
    /// @brief Valid roi on parsing img(not camera image)
    Rect roi_output;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit IpmParsingData(std::int64_t timestamp) noexcept;

  IpmParsingData() noexcept;
  ~IpmParsingData() noexcept final = default;
  IpmParsingData(IpmParsingData const&) = default;
  IpmParsingData& operator=(IpmParsingData const&) = default;
  IpmParsingData(IpmParsingData&&) = default;
  IpmParsingData& operator=(IpmParsingData&&) = default;
  /// @brief Get Ipm Parsing data
  ///
  /// @return ipm_parsing_data_
  IpmParsing const& GetIpmParsing() const noexcept;

  /// @brief Mutable Ipm Parsing data
  ///
  /// @return ipm_parsing_data_
  IpmParsing& MutableIpmParsing() noexcept;

  /// @brief Set Ipm Parsing data
  ///
  /// @param ipm_parsing_data Ipm Parsing data
  void SetIpmParsing(IpmParsing const& ipm_parsing_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  IpmParsing ipm_parsing_data_;
};

}  // namespace autodrive

#endif
