//
#ifndef AUTODRIVE_INTERFACE_USS_PARKING_SLOTS_DATA_H_
#define AUTODRIVE_INTERFACE_USS_PARKING_SLOTS_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/uss_base.h"

namespace autodrive {

class UssParkingSlotsData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxUssPsSize{4U};
  static constexpr std::uint32_t kMaxPerFusUssRawDataEchoSize{6U};
  static constexpr std::uint32_t kMaxObjFrameUssObjPointsSize{50U};
  static constexpr std::uint32_t kMaxUssHeaderSize{2U};

  /// @brief Parking spot mode
  enum class UssPsStatusEn : std::uint8_t {
    /// @brief Default invalid value
    kUssPsStatusEnNone = 0,
    /// @brief Search mode
    kUssPsStatusEnSearch = 1,
    /// @brief Correction mode
    kUssPsStatusEnCorret = 2,
    kUssPsStatusEnMax = 3
  };

  /// @brief Parking spot type
  enum class UssPsTypeEn : std::uint8_t {
    /// @brief Default invalid value
    kUssPsTypeEnNone = 0,
    /// @brief Horizontal parking spot
    kUssPsTypeEnParallel = 1,
    /// @brief Vertical parking spot
    kUssPsTypeEnCross = 2,
    /// @brief Diagonal parking spot (only for correction mode)
    kUssPsTypeEnDiagonal = 3,
    kUssPsTypeEnMax = 4
  };

  /// @brief Parking spot orientation
  enum class UssPsOrientationEn : std::uint8_t {
    /// @brief Default invalid value
    kUssPsOrientationEnNone = 0,
    /// @brief Parking spot on the left side of the vehicle
    kUssPsOrientationEnLeft = 1,
    /// @brief Parking spot on the right side of the vehicle
    kUssPsOrientationEnRight = 2,
    /// @brief Maximum identifier for the enumeration
    kUssPsOrientationEnMax = 3
  };

  /// @brief Parking spot boundary properties
  enum class UssPsBoundaryTypeEn : std::uint8_t {
    /// @brief Virtual boundary for correction
    kPerFusUssPsBoundaryTypeVirtual = 0,
    /// @brief Low obstacle
    kPerFusUssPsBoundaryTypeLow = 1,
    /// @brief High obstacle
    kPerFusUssPsBoundaryTypeHigh = 2,
    /// @brief Wall
    kPerFusUssPsBoundaryTypeWall = 3,
    /// @brief Curb
    kPerFusUssPsBoundaryTypeCurb = 4,
    kPerFusUssPsBoundaryTypeMax = 5
  };

  /// @brief Ultrasonic parking spot, Ultrasonic correction parking spot
  struct alignas(8) PerFusUssPs {
    /// @brief Uss parking spot - spot ID
    uint8_t uss_ps_id_ui8;
    /// @brief Uss parking spot - spot status
    UssPsStatusEn uss_ps_status_en;
    /// @brief Uss parking spot - spot type
    UssPsTypeEn uss_ps_type_en;
    /// @brief Uss parking spot - spot orientation
    UssPsOrientationEn uss_ps_orientation_en;

    /// @brief Uss parking spot - boundary 1 property (AD line)
    UssPsBoundaryTypeEn uss_ps_1st_boundary_type_en;
    /// @brief Uss parking spot - boundary 2 property (BC line)
    UssPsBoundaryTypeEn uss_ps_2nd_boundary_type_en;
    /// @brief Uss correction parking spot - cross boundary property (CD line)
    /// based on standard scene
    UssPsBoundaryTypeEn uss_ps_cross_boundary_type_en;
    std::uint8_t padding_1[1];

    /// @brief Uss parking spot - spot depth in mm; default 5000mm for vertical
    /// spot
    std::uint16_t uss_ps_depth_ui16;
    /// @brief Uss parking spot - spot length in mm
    std::uint16_t uss_ps_length_ui16;
    /// @brief Uss parking spot - spot angle
    std::int16_t uss_ps_angle_i16;
    std::uint8_t padding_2[2];

    /// @brief Uss parking spot - spot coordinate 1 in mm
    UssPointSi32 uss_ps_1st_corner;
    /// @brief Uss parking spot - spot coordinate 2 in mm
    UssPointSi32 uss_ps_2nd_corner;
    /// @brief Uss parking spot - spot coordinate 3 in mm
    UssPointSi32 uss_ps_3st_corner;
    /// @brief Uss parking spot - spot coordinate 4 in mm
    UssPointSi32 uss_ps_4nd_corner;
    UssPointSi32 uss_ps_1st_corner_gcs;
    UssPointSi32 uss_ps_2nd_corner_gcs;
    UssPointSi32 uss_ps_3st_corner_gcs;
    UssPointSi32 uss_ps_4nd_corner_gcs;

    /// @brief USS parking spot - parking spot boundary information point 1 in
    /// mm
    UssPointSi16 uss_ps_boundary_1st_info;
    /// @brief USS parking spot - parking spot boundary information point 2 in
    /// mm
    UssPointSi16 uss_ps_boundary_2nd_info;
    /// @brief USS parking spot - completion timestamp
    std::int64_t uss_ps_finish_timestamp;
  };

  struct alignas(8) UssParkingSlots {
    std::int32_t version;
    std::uint16_t num_uss_header;
    std::uint16_t num_uss_ps;
    std::array<PerFusUssHeader, kMaxUssHeaderSize> uss_header;
    std::array<PerFusUssPs, kMaxUssPsSize> uss_ps;
    PerFusUssPs uss_correct_ps;
    StateMachine state_machine;
    std::int64_t sensor_stamp;
    std::int64_t sample_stamp;
    std::uint64_t frame_id;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit UssParkingSlotsData(std::int64_t timestamp) noexcept;

  UssParkingSlotsData() noexcept;
  ~UssParkingSlotsData() noexcept final = default;
  UssParkingSlotsData(UssParkingSlotsData const&) = default;
  UssParkingSlotsData& operator=(UssParkingSlotsData const&) = default;
  UssParkingSlotsData(UssParkingSlotsData&&) = default;
  UssParkingSlotsData& operator=(UssParkingSlotsData&&) = default;

  /// @brief Get Uss Parking Slots
  ///
  /// @return uss_parking_slots_
  UssParkingSlots const& GetUssParkingSlots() const noexcept;

  /// @brief Mutable Uss Parking Slots
  ///
  /// @return uss_parking_slots_
  UssParkingSlots& MutableUssParkingSlots() noexcept;

  /// @brief Set Uss Parking Slots
  ///
  /// @param uss_parking_slots sync Uss Parking Slots
  void SetUssParkingSlots(UssParkingSlots const& uss_parking_slots) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  UssParkingSlots uss_parking_slots_;
};

}  // namespace autodrive

#endif
