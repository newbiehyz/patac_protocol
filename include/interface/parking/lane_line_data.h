//
#ifndef AUTODRIVE_INTERFACE_LANE_LINE_DATA_H_
#define AUTODRIVE_INTERFACE_LANE_LINE_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/basic.h"

namespace autodrive {

class LaneLineData final : public CommonMsgData {
 public:
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit LaneLineData(std::int64_t timestamp) noexcept;

  struct LaneLinePoint{
    Vector2f line_start_point_vcs;
    Vector2f line_end_point_vcs;
    /// line flag   0:invalid  1:valid
    uint8_t flag;
    uint8_t padding[7];
  };

  struct LaneLinePoints{
    LaneLinePoint near_line;
    LaneLinePoint far_line;
  };

  LaneLineData() noexcept;
  ~LaneLineData() noexcept final = default;
  LaneLineData(LaneLineData const&) = default;
  LaneLineData& operator=(LaneLineData const&) = default;
  LaneLineData(LaneLineData&&) = default;
  LaneLineData& operator=(LaneLineData&&) = default;

  /// @brief Get Lane Line Points
  ///
  /// @return lane_line_points_
  LaneLinePoints const& GetLaneLinePoints() const noexcept;

  /// @brief Mutable Lane Line Points
  ///
  /// @return lane_line_points_
  LaneLinePoints& MutableLaneLinePoints() noexcept;

  /// @brief Set Lane Line Points
  ///
  /// @param lane_line_points Lane Line Points
  void SetLaneLinePoints(LaneLinePoints const& lane_line_points) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  LaneLinePoints lane_line_points_;
};

}  // namespace autodrive

#endif // AUTODRIVE_INTERFACE_LANE_LINE_DATA_H_