//
#ifndef AUTODRIVE_INTERFACE_SPEED_BUMP_DATA_H_
#define AUTODRIVE_INTERFACE_SPEED_BUMP_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/basic.h"

namespace autodrive {

class SpeedBumpData final : public CommonMsgData {
 public:
  static constexpr std::uint8_t kMaxSpeedBumpSize{20};

  struct alignas(8) SpeedBump{
    uint8_t speed_bump_id;
    std::uint8_t padding[7];
    Vector2f p_0_vcs;
    Vector2f p_1_vcs;
    Vector2f p_2_vcs;
    Vector2f p_3_vcs;
  };

  struct alignas(8) SpeedBumps{
    uint8_t speed_bump_nums;
    std::uint8_t padding[7];
    std::array<SpeedBump,kMaxSpeedBumpSize> speed_bump_list;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit SpeedBumpData(std::int64_t timestamp) noexcept;

  SpeedBumpData() noexcept;
  ~SpeedBumpData() noexcept final = default;
  SpeedBumpData(SpeedBumpData const&) = default;
  SpeedBumpData& operator=(SpeedBumpData const&) = default;
  SpeedBumpData(SpeedBumpData&&) = default;
  SpeedBumpData& operator=(SpeedBumpData&&) = default;

  /// @brief Get Speed Bumps
  ///
  /// @return speed_bumps_
  SpeedBumps const& GetSpeedBumps() const noexcept;

  /// @brief Mutable Speed Bumps
  ///
  /// @return speed_bumps_
  SpeedBumps& MutableSpeedBumps() noexcept;

  /// @brief Set Speed Bumps
  ///
  /// @param speed_bumps Speed Bumps data
  void SetSpeedBumps(SpeedBumps const& speed_bumps) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  SpeedBumps speed_bumps_;
};

}  // namespace autodrive

#endif // AUTODRIVE_INTERFACE_SPEED_BUMP_DATA_H_