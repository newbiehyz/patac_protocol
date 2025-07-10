//
#ifndef AUTODRIVE_INTERFACE_FUSION_FREESPACE_DATA_H_
#define AUTODRIVE_INTERFACE_FUSION_FREESPACE_DATA_H_

#include <foundation/type/common_msg_data.h>
#include <array>
#include <type/basic.h>

namespace autodrive {

class FusionFreespaceData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxFreespacePointsSize{460U};

  enum class FreespacePointLabel : std::uint8_t {
    kRoad = 0,
    kParkingLockOpen = 11,
    kParkingLockClosed = 12,
    kTrafficCone = 13,
    kParkingRod = 14,
    kCurb = 15,
    kCementColumn = 16,
    kImmovableObstacle = 17,
    kMovableObstacle = 18,
    kBackground = 19,
    kSidewalk = 20,
    kPaintedWallRoot = 21,
    kDynamicObstacle = 22,
    kUssObj = 27,
    kUssCrossPoint = 28,
    kUnknwn = 100,
    kInvalid = 255
  };

  struct alignas(8) FreespacePoint {
    /// @brief ReservedFields
    float conf;
    /// @brief Freespace point index
    float angle;
    /// @brief Distance between freespace point and vehicle rear axis mid point
    float ray_length;
    /// @brief Identical with label
    std::uint32_t origin_label;
    /// @brief Freespace point label
    FreespacePointLabel label;
    std::uint8_t padding[7];
    /// @brief Freespace point coordinate at VCS
    Vector2f pt_vcs;
    /// @brief ReservedFields
    Vector2f vel_vcs;
    /// @brief Freespace point coordinate at WCS
    Vector2f pt_world;
    /// @brief ReservedFields
    Vector2f vel_world;
  };

  struct alignas(8) FreespaceFusion {
    std::int64_t timestamp;
    std::uint32_t num_freespace_points;
    std::uint8_t padding[4];
    std::array<FreespacePoint, kMaxFreespacePointsSize> freespace_points;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit FusionFreespaceData(std::int64_t timestamp) noexcept;

  FusionFreespaceData() noexcept;
  ~FusionFreespaceData() noexcept final = default;
  FusionFreespaceData(FusionFreespaceData const&) = default;
  FusionFreespaceData& operator=(FusionFreespaceData const&) = default;
  FusionFreespaceData(FusionFreespaceData&&) = default;
  FusionFreespaceData& operator=(FusionFreespaceData&&) = default;

  /// @brief Get Fusion Freespace data
  ///
  /// @return fusion_freespace_data_
  FreespaceFusion const& GetFusionFreespace() const noexcept;

  /// @brief Mutable Fusion Freespace data
  ///
  /// @return fusion_freespace_data_
  FreespaceFusion& MutableFusionFreespace() noexcept;

  /// @brief Set Fusion Freespace data
  ///
  /// @param fusion_freespace_data fusion result data
  void SetFusionFreespace(FreespaceFusion const& fusion_freespace_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  FreespaceFusion fusion_freespace_data_;
};

}  // namespace autodrive

#endif
