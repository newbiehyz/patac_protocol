//
#ifndef AUTODRIVE_INTERFACE_USS_OBSTACLES_DATA_H_
#define AUTODRIVE_INTERFACE_USS_OBSTACLES_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/uss_base.h"

namespace autodrive {

class UssObstaclesData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxPerFusUssRawDataEchoSize{6U};
  static constexpr std::uint32_t kMaxObjFrameUssObjPointsSize{50U};
  static constexpr std::uint32_t kMaxUssHeaderSize{2U};
  static constexpr std::uint32_t kMaxUssRawSize{12U};
  static constexpr std::uint32_t kMaxUssObjSize{45U};

  /// @brief
  enum class PerFusSnsID : std::uint8_t {
    kPerFusSnsIDNone = 0,
    kPerFusSnsID1 = 1,
    kPerFusSnsID2 = 2,
    kPerFusSnsID3 = 3,
    kPerFusSnsID4 = 4,
    kPerFusSnsID5 = 5,
    kPerFusSnsID6 = 6,
    kPerFusSnsID7 = 7,
    kPerFusSnsID8 = 8,
    kPerFusSnsID9 = 9,
    kPerFusSnsID10 = 10,
    kPerFusSnsID11 = 11,
    kPerFusSnsID12 = 12,
    kPerFusSnsIDMax
  };

  /// @brief Receiving mode
  enum class PerFusEchoMode : std::uint8_t {
    /// @brief Default invalid value
    kPerFusEchoModeNone = 0,
    /// @brief DE
    kPerFusEchoModeDE = 1,
    /// @brief CE
    kPerFusEchoModeCE = 2,
    kPerFusEchoModeMax
  };

  struct alignas(8) PerFusUssRawData {
    std::uint8_t num_echo;
    /// @brief Bind zone flag
    std::uint8_t bind_zone_flag;
    std::uint8_t padding_1[2];

    /// @brief Ring time
    std::uint16_t ring_time;
    /// @brief Bind zone distance in mm
    std::uint16_t bind_zone_dis;
    /// @brief Time difference between transmission and reception of ultrasound
    /// waves (not recommended for use, as the time interval is too small in
    /// microseconds to obtain accurate time)
    std::uint16_t time_diff_tx_rx;
    /// @brief Environmental noise
    std::uint16_t env_noise;

    /// @brief Distance of echoes 1-6 in mm
    std::array<std::uint16_t, kMaxPerFusUssRawDataEchoSize> echo_dist;
    /// @brief Confidence of echoes 1-6 (variable frequency has confidence,
    /// fixed frequency defaults to 0, range 0-15, with higher values indicating
    /// higher confidence)
    std::array<std::uint16_t, kMaxPerFusUssRawDataEchoSize> echo_cfd;
    /// @brief Height of echoes 1-6
    std::array<std::uint16_t, kMaxPerFusUssRawDataEchoSize> echo_high;
    /// @brief Width of echoes 1-6 (reserved for TTE probes that are not fully
    /// developed yet)
    std::array<std::uint16_t, kMaxPerFusUssRawDataEchoSize> echo_width;
    /// @brief Measurement time of echoes 1-6 (time from transmission to closing
    /// of the probe)
    std::array<std::uint16_t, kMaxPerFusUssRawDataEchoSize> echo_mt;

    /// @brief timestamp of echo transmission
    std::uint64_t echo_send_timestamp;

    /// @brief Sensor id
    PerFusSnsID per_fus_sns_id;
    /// @brief Receiving mode
    PerFusEchoMode per_fus_echo_mode;
    /// @brief Source probe of echo (variable frequency)
    PerFusSnsID per_fus_echo_source_id;

    std::uint8_t padding_2[5];
  };

  struct alignas(8) PerFusUssAllCERawData {
    std::uint16_t ce1_to_2distance;
    std::uint16_t ce2_to_1distance;
    std::uint16_t ce2_to_3distance;
    std::uint16_t ce3_to_2distance;
    std::uint16_t ce3_to_4distance;
    std::uint16_t ce4_to_3distance;
    std::uint16_t ce4_to_5distance;
    std::uint16_t ce5_to_4distance;
    std::uint16_t ce5_to_6distance;
    std::uint16_t ce6_to_5distance;
    std::uint16_t ce7_to_8distance;
    std::uint16_t ce8_to_7distance;
    std::uint16_t ce8_to_9distance;
    std::uint16_t ce9_to_8distance;
    std::uint16_t ce9_to_10distance;
    std::uint16_t ce10_to_9distance;
    std::uint16_t ce10_to_11distance;
    std::uint16_t ce11_to_10distance;
    std::uint16_t ce11_to_12distance;
    std::uint16_t ce12_to_11distance;
    std::int64_t ce1_to_2timestamp;
    std::int64_t ce2_to_1timestamp;
    std::int64_t ce2_to_3timestamp;
    std::int64_t ce3_to_2timestamp;
    std::int64_t ce3_to_4timestamp;
    std::int64_t ce4_to_3timestamp;
    std::int64_t ce4_to_5timestamp;
    std::int64_t ce5_to_4timestamp;
    std::int64_t ce5_to_6timestamp;
    std::int64_t ce6_to_5timestamp;
    std::int64_t ce7_to_8timestamp;
    std::int64_t ce8_to_7timestamp;
    std::int64_t ce8_to_9timestamp;
    std::int64_t ce9_to_8timestamp;
    std::int64_t ce9_to_10timestamp;
    std::int64_t ce10_to_9timestamp;
    std::int64_t ce10_to_11timestamp;
    std::int64_t ce11_to_10timestamp;
    std::int64_t ce11_to_12timestamp;
    std::int64_t ce12_to_11timestamp;
  };

  /// @brief Anchor point properties
  enum class LpAttributesEn : std::uint8_t {
    kLpAttributesEnNone = 0,
    kLpAttributesEnSure = 1,
    kLpAttributesEnUnsure = 2,
    kLpAttributesEnUnknown = 3,
    kLpAttributesEnMax = 4
  };

  /// @brief The anchor point is at the position of the vehicle
  enum class LpDirEn : std::uint8_t {
    kLpDirEnNone = 0,
    kLpDirEnFront = 1,
    kLpDirEnRear = 2,
    kLpDirEnLeft = 3,
    kLpDirEnRight = 4,
    kLpDirEnInside = 5,
    kLpDirEnMax = 6
  };

  /// @brief Anchor point probe combination
  enum class LpSnsGroupIdEn : std::uint8_t {
    kLpSnsGroupIdEnNone = 0,
    kLpSnsGroupIdEn1_2 = 1,
    kLpSnsGroupIdEn2_3 = 2,
    kLpSnsGroupIdEn3_4 = 3,
    kLpSnsGroupIdEn4_5 = 4,
    kLpSnsGroupIdEn5_6 = 5,
    kLpSnsGroupIdEn11_12 = 6,
    kLpSnsGroupIdEn10_11 = 7,
    kLpSnsGroupIdEn9_10 = 8,
    kLpSnsGroupIdEn8_9 = 9,
    kLpSnsGroupIdEn7_8 = 10,
    kLpSnsGroupIdEnMax = 11
  };

  /// @brief Location point type
  enum class LpTypeEn : std::uint8_t {
    /// @brief Default invalid value
    kLpTypeEnNone = 0,
    /// @brief Single DE obstacle location
    kLpTypeEnDe = 1,
    /// @brief Double DE obstacle location
    kLpTypeEnDeDe = 2,
    /// @brief Single CE obstacle location
    kLpTypeEnCe = 3,
    /// @brief DE+CE obstacle location
    kLpTypeEnDeCe = 4,
    /// @brief Column (circular surface)
    kLpTypeEnRound = 5,
    /// @brief Other
    kLpTypeEnOther = 6,
    kLpTypeEnMax = 7
  };

  /// @brief Ultrasonic localization point - the middle value that generates
  /// obstacles, only representing the possible points currently perceived
  struct alignas(8) PerFusUssLp {
    /// @brief USS location point - LpId
    std::uint8_t lp_id_ui8;
    /// @brief USS location point - DE echo count
    std::uint8_t lp_de_echo_cnt_ui8;
    /// @brief USS location point - CE echo count
    std::uint8_t lp_ce_echo_cnt_ui8;
    /// @brief USS location point - location point attributes
    LpAttributesEn lp_attributes_en;

    /// @brief USS location point - distance from obstacle to sensor A in mm
    std::uint16_t lp_dist_sens_a_ui16;
    /// @brief USS location point - distance from obstacle to sensor B in mm
    std::uint16_t lp_dist_sens_b_ui16;
    /// @brief USS location point - lateral error L1 in mm
    std::uint16_t lp_corr_l1_ui16;
    /// @brief USS location point - longitudinal error L2 in mm
    std::uint16_t lp_corr_l2_ui16;

    /// @brief USS location point - located in the direction of the vehicle
    LpDirEn lp_dir_en;
    /// @brief USS location point - probe combination
    LpSnsGroupIdEn lp_sns_group_id_en;
    /// @brief USS location point - location point type
    LpTypeEn lp_type_en;
    std::uint8_t padding[1];

    /// @brief USS location point - completion timestamp
    std::int64_t lp_id_finish_timestamp_si64;

    /// @brief USS location point - possible point 1 in mm
    UssPointSi16 lp_pos_1_st;
    /// @brief USS location point - possible point 2 in mm
    UssPointSi16 lp_pos_2_st;
    /// @brief USS location point - possible point 3 in mm
    UssPointSi16 lp_pos_3_st;
  };

  /// @brief Obstacle point type
  enum class UssObjXTypeEn : std::uint8_t {
    /// @brief Default invalid value
    kUssObjXTypeEnNone = 0,
    /// @brief Real point-like obstacle (starting point and ending point
    /// together)
    kUssObjXTypeEnPoint = 1,
    /// @brief Straight obstacle with undetermined starting and ending points
    kUssObjXTypeEnStraight0C = 2,
    /// @brief Straight obstacle with undetermined starting or ending point
    kUssObjXTypeEnStraight1C = 3,
    /// @brief Straight obstacle with determined starting and ending points
    kUssObjXTypeEnStraight2C = 4,
    kUssObjXTypeEnMax = 5
  };

  /// @brief Obstacle height information
  enum class UssObjXHeightEn : std::uint8_t {
    /// @brief Default invalid value
    kUssObjXHeightEnNone = 0,
    /// @brief Low height < 30CM
    kUssObjXHeightEnLow = 1,
    /// @brief High height > 1M (estimated, also related to sensor installation
    /// height)
    kUssObjXHeightEnHigh = 2,
    /// @brief Height unknown, other distance
    kUssObjXHeightEnUnknow = 3,
    kUssObjXHeightEnMax = 4
  };

  /// @brief Whether the obstacle is on the path
  enum class UssObjXOnCourseEn : std::uint8_t {
    /// @brief Default invalid value
    kUssObjXOnCourseEnNone = 0,
    /// @brief On the path
    kUssObjXOnCourseEnOnCourse = 1,
    /// @brief Not on the path
    kUssObjXOnCourseEnNotOnCourse = 2,
    kUssObjXOnCourseEnMax = 3
  };

  /// @brief Ultrasonic obstacle
  struct alignas(8) PerFusUssHisObjFrame {
    std::uint8_t num_uss_obj_points;
    std::uint8_t padding_1[3];

    /// @brief USS obstacle - cluster historical location point type
    std::array<LpTypeEn, kMaxObjFrameUssObjPointsSize> lp_type_en;
    std::uint8_t padding_2[2];

    /// @brief USS obstacle - cluster historical location point completion
    /// timestamp
    std::array<std::int64_t, kMaxObjFrameUssObjPointsSize>
        finish_timestamp_si64;
    /// @brief USS obstacle - cluster historical location point information
    /// [point coordinates] up to 50 units in mm
    std::array<UssPointSi16, kMaxObjFrameUssObjPointsSize> uss_obj_points;
  };

  struct alignas(8) PerFusUssObj {
    /// @brief USS obstacle - obstacle ID
    std::uint8_t uss_obj_x_id_ui8;
    /// @brief USS obstacle - obstacle confidence
    std::uint8_t uss_obj_x_cfd_ui8;
    /// @brief USS obstacle - obstacle point type
    UssObjXTypeEn uss_obj_x_type_en;

    /// @brief USS obstacle - obstacle height information
    UssObjXHeightEn uss_obj_x_height_en;
    /// @brief USS obstacle - whether the obstacle is on the path, without this
    /// function in perception, not recommended to use, format reserved
    /// temporarily
    UssObjXOnCourseEn uss_obj_x_on_course_en;
    std::uint8_t objHistory_ui8;
    std::uint8_t padding[2];

    /// @brief Completion timestamp
    std::int64_t finish_timestamp_si64;

    /// @brief USS obstacle - obstacle coordinate 1 in mm
    UssPointSi16 uss_obj_x_pos_1;
    /// @brief USS obstacle - obstacle coordinate 2 in mm
    UssPointSi16 uss_obj_x_pos_2;
    /// @brief USS obstacle - obstacle error 1 in mm
    UssPointSi16 uss_obj_x_err_pos_1;
    /// @brief USS obstacle - obstacle error 2 in mm
    UssPointSi16 uss_obj_x_err_pos_2;

  };

  struct alignas(8) UssObstacles {
    std::int32_t version;
    std::uint8_t num_uss_header;
    std::uint8_t num_uss_raw;
    std::uint8_t padding;
    std::uint8_t num_uss_obj;
    std::array<PerFusUssHeader, kMaxUssHeaderSize> uss_header;
    std::array<PerFusUssRawData, kMaxUssRawSize> uss_raw;
    PerFusUssAllCERawData uss_ce_raw_obj;
    std::array<PerFusUssObj, kMaxUssObjSize> uss_obj;
    StateMachine state_machine;
    std::int64_t sensor_stamp;
    std::int64_t sample_stamp;
    std::uint64_t frame_id;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit UssObstaclesData(std::int64_t timestamp) noexcept;

  UssObstaclesData() noexcept;
  ~UssObstaclesData() noexcept final = default;
  UssObstaclesData(UssObstaclesData const&) = default;
  UssObstaclesData& operator=(UssObstaclesData const&) = default;
  UssObstaclesData(UssObstaclesData&&) = default;
  UssObstaclesData& operator=(UssObstaclesData&&) = default;

  /// @brief Get Uss Obstacles
  ///
  /// @return uss_obstacles_
  UssObstacles const& GetUssObstaclesData() const noexcept;

  /// @brief Mutable Uss Obstacles
  ///
  /// @return uss_obstacles_
  UssObstacles& MutableUssObstaclesData() noexcept;

  /// @brief Set Uss Obstacles
  ///
  /// @param uss_obstacles sync Uss Obstacles
  void SetUssObstaclesData(UssObstacles const& uss_obstacles) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  UssObstacles uss_obstacles_;
};

}  // namespace autodrive

#endif
