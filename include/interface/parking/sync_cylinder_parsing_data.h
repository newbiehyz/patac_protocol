//
#ifndef AUTODRIVE_INTERFACE_SYNC_CYLINDER_PARSING_DATA_H_
#define AUTODRIVE_INTERFACE_SYNC_CYLINDER_PARSING_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/parsing_base.h"

namespace autodrive {

class SyncCylinderParsingData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxCylinderParsingDataSize{4U};

  enum class CylinderParsingLabel : std::uint8_t {
    /// @brief Road
    kRoad = 0,
    /// @brief Sidewalk
    kSidewalk = 1,
    /// @brief Sky
    kSky = 2,
    /// @brief Terrain
    kTerrain = 3,
    /// @brief Curb
    kCurb = 4,
    /// @brief Fence
    kFence = 5,
    /// @brief Tree
    kTree = 6,
    /// @brief Pole
    kPole = 7,
    /// @brief Traffic Sign
    kTrafficSign = 8,
    /// @brief Vehicle
    kVehicle = 9,
    /// @brief Three Wheel
    kThreeWheel = 10,
    /// @brief Two Wheel
    kTwoWheel = 11,
    /// @brief Person
    kPerson = 12,
    /// @brief Rider
    kRider = 13,
    /// @brief Trolley
    kTrolley = 14,
    /// @brief Traffic Cone
    kTrafficCone = 15,
    /// @brief Bollard
    kBollard = 16,
    /// @brief Folding Warning Sign
    kFoldingWarningSign = 17,
    /// @brief Single Water Barrier
    kSingleWaterBarrier = 18,
    /// @brief Untraversable
    kUntraversable = 19,
    /// @brief Parking Rod
    kParkingRod = 20,
    /// @brief Parking Lock
    kParkingLock = 21,
    /// @brief Cement Column
    kCementColumn = 22,
    /// @brief background
    kbackground = 23,
    /// @brief Fire Hydrant
    kFireHydrant = 24,
    /// @brief Painted Wall Root
    kPaintedWallRoot = 25
  };

  struct alignas(8) CylinderParsingData {
    /// @brief Camera id
    std::uint8_t camera_idx;
    std::uint8_t padding[3];
    /// @brief Parsing scale comparing to camera img width
    float scale;

    /// @brief Model output Parsing
    CylParsingImageData feature_map;
    /// @brief Rle compressed parsing results
    RleCompressedData rle_feature_map;
    /// @brief Valid roi on parsing img(not camera image)
    Rect roi_output;
  };

  struct alignas(8) SyncCylinderParsing {
    std::uint8_t padding[4];
    /// @brief Parsing num
    std::uint32_t num_cylinder_parsing_data;
    /// @brief Parsing array
    std::array<CylinderParsingData, kMaxCylinderParsingDataSize>
        cylinder_parsing_data;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit SyncCylinderParsingData(std::int64_t timestamp) noexcept;

  SyncCylinderParsingData() noexcept;
  ~SyncCylinderParsingData() noexcept final = default;
  SyncCylinderParsingData(SyncCylinderParsingData const&) = default;
  SyncCylinderParsingData& operator=(SyncCylinderParsingData const&) = default;
  SyncCylinderParsingData(SyncCylinderParsingData&&) = default;
  SyncCylinderParsingData& operator=(SyncCylinderParsingData&&) = default;

  /// @brief Get Sync Cylinder Parsing data
  ///
  /// @return sync_cylinder_parsing_data_
  SyncCylinderParsing const& GetSyncCylinderParsing() const noexcept;

  /// @brief Mutable Sync Cylinder Parsing data
  ///
  /// @return sync_cylinder_parsing_data_
  SyncCylinderParsing& MutableSyncCylinderParsing() noexcept;

  /// @brief Set Sync Cylinder Parsing data
  ///
  /// @param sync_cylinder_parsing_data sync cylinder parsing data
  void SetSyncCylinderParsing(SyncCylinderParsing const& sync_cylinder_parsing_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  SyncCylinderParsing sync_cylinder_parsing_data_;
};

}  // namespace autodrive

#endif
