//

#ifndef AUTODRIVE_INTERFACE_IPM_MULTITASK_DATA_H_
#define AUTODRIVE_INTERFACE_IPM_MULTITASK_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/basic.h"
#include "type/parsing_base.h"

namespace autodrive {

class IpmMultitaskData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxParkingSlotRawSize{30U};
  static constexpr std::uint32_t kParkingSlotPointSize{4U};
  static constexpr std::uint32_t kMaxIpmWheelGroundPointSize{50U};

  enum class ParkingSlotPointType : std::uint8_t {
    kTruncated = 0,
    kCorner = 1,
  };

  enum class ParkingSlotType : std::uint8_t {
    kUnknown = 0,
    kVertical = 1,
    kParallel = 2,
    kOblique = 3,
  };

  struct alignas(8) ParkingSlotPoint {
    Vector2f point;
    float score;
    ParkingSlotPointType point_type;
    bool visibility;
    std::uint8_t padding[2];
    float point_orientation_x;
    float point_orientation_y;
  };

  struct alignas(8) ParkingSlotRaw {
    std::array<ParkingSlotPoint, kParkingSlotPointSize> slot_points;
    float score;
    std::int8_t corner_match_flag;
    bool occupancy;
    ParkingSlotType slot_type;
    std::uint8_t padding[1];
    float slot_orientation_x;
    float slot_orientation_y;
  };

  struct alignas(8) IpmWheelGroundPoint{
    Vector2f point;
    float score;
    std::uint8_t padding[4];
  };

  IpmMultitaskData() noexcept;
  IpmMultitaskData(std::int64_t timestamp) noexcept;
  ~IpmMultitaskData() noexcept final = default;

  /// @brief Get slots_raw_
  ///
  /// @return slots_raw_
  std::array<ParkingSlotRaw, kMaxParkingSlotRawSize> const &GetSlotsRaw()
      const noexcept;

  /// @brief Mutable slots_raw_
  ///
  /// @return slots_raw_
  std::array<ParkingSlotRaw, kMaxParkingSlotRawSize> &
  MutableSlotsRaw() noexcept;

  /// @brief Set slots_raw_
  ///
  /// @return void
  void SetSlotsRaw(std::array<ParkingSlotRaw, kMaxParkingSlotRawSize> const
                       &slots_raw) noexcept;

  /// @brief get num_slots_raw_
  ///
  /// return uint8_t
  void SetSlotsNum(const std::uint8_t& slots_num) noexcept;

  /// @brief get num_slots_raw_
  ///
  /// return uint8_t
  uint8_t const &GetSlotsNum() const noexcept;

  /// @brief get num_slots_raw_
  ///
  /// return uint8_t
  uint8_t &MutableSlotsNum() noexcept;

  /// @brief Get ipm_wheel_ground_points_
  ///
  /// @return ipm_wheel_ground_points_
  std::array<IpmWheelGroundPoint, kMaxIpmWheelGroundPointSize> const &GetIpmWheelGroundPoints()
      const noexcept;

  /// @brief ipm_wheel_ground_points_;
  ///
  /// @return void
  void SetIpmWheelGroundPoints(std::array<IpmWheelGroundPoint, kMaxIpmWheelGroundPointSize> const
                       &ipm_wheel_ground_points) noexcept;

  /// @brief ipm_wheel_ground_points_
  ///
  /// @return ipm_wheel_ground_points_
  std::array<IpmWheelGroundPoint, kMaxIpmWheelGroundPointSize> &
    MutableIpmWheelGroundPoints() noexcept;

  /// @brief get num_wheel_gdp_
  ///
  /// return uint8_t
  uint8_t const &GetWheelGDPNum() const noexcept;

  /// @brief get num_wheel_gdp_
  ///
  /// return uint8_t
  void SetWheelGDPNum(const std::uint8_t& wheel_gdp_num) noexcept;

  /// @brief num_wheel_gdp_
  ///
  /// return uint8_t
  uint8_t &MutableWheelGDPNum() noexcept;



  float const &GetTest() const noexcept;
  float &MutableTest() noexcept;
  void SetTest(const float& confidence) noexcept;



  /// @brief Clear data
  void Clear() noexcept;

 private:
  uint8_t num_slots_raw_;
  std::array<ParkingSlotRaw, kMaxParkingSlotRawSize> slots_raw_;
  uint8_t num_wheel_gdp_;
  std::array<IpmWheelGroundPoint, kMaxIpmWheelGroundPointSize> ipm_wheel_ground_points_;
  float test_;
};
}  // namespace autodrive

#endif