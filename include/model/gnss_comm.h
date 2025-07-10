//
#ifndef AUTODRIVE_MODEL_COMMUNICATION_GNSS_COMM_H_
#define AUTODRIVE_MODEL_COMMUNICATION_GNSS_COMM_H_

#include <cstdint>

namespace autodrive {
namespace gnss_comm {

/// @brief Max satellite info number
static constexpr std::uint8_t kMaxSatInfoNum{36U};

// UTC time
struct alignas(8) UtcTime {
  /// @brief Year
  std::uint8_t year;
  /// @brief Month
  std::uint8_t month;
  /// @brief Day
  std::uint8_t day;
  /// @brief Hour
  std::uint8_t hour;
  /// @brief Minute
  std::uint8_t minute;
  /// @brief Second
  std::uint8_t second;
  /// @brief Millisecond
  std::uint16_t milli_second;
  /// @brief Time accuracy(ns)
  std::uint16_t time_accuracy;
  /// @brief  padding
  std::uint8_t padding[6];
};

// Position status
enum class PositionStatus : std::uint8_t {
  INVALID = 0,          // Invalid
  DR = 1,               // DR
  FIXED_2D = 2,         // 2D fixed
  FIXED_3D = 3,         // 3D fixed
  GNSS_DR = 4,          // GNSS + DR
  ONLY_TIME_VALID = 5   // Only time valid
};

// Positing system
enum class PositingSystem : std::uint8_t {
  GPS = 0,     // GPS
  BEI_DOU = 1, // Bei dou
  GLONASS = 2, // Glonass
  GALILEO = 3, // Galileo
  QZSS = 4,    // Qzss
  SBAS = 5,    // Sbas
  UNKNOWN = 6  // Unknown
};

// GNSS satellite info
struct alignas(8) SatelliteInfo {
  /// @brief Num in view (GnssNumInView)
  std::uint8_t num_in_view;
  /// @brief No (GnssSatPrn)
  std::uint8_t no;
  /// @brief Snr (GnssSatSnr)
  std::uint8_t snr;
  /// @brief Elevation(deg) (GnssSatElev)
  std::uint8_t elevation;
  /// @brief Azimuth(deg) (GnssSatAzimuth)
  std::uint16_t azimuth;
  /// @brief Positing system (GnssSatSystem)
  PositingSystem positing_system;
  /// @brief padding
  std::uint8_t padding;
};

enum class PoseType : std::uint8_t {
  INVALID = 0,          // Invalid
  SINGLE = 1,           // Single
  DGPS = 2,             // DGPS
  RTK_FIXED = 4,        // Rtk fixed
  RTK_FLOAT = 5,        // Rtk float
  DEAD_RECKONING = 6    // Dead reckoning
};

// GNSS
struct alignas(8) Gnss {
  /// @brief GNSS time (GNSS_TimeStamp)
  UtcTime utc_time;
  /// @brief Latitude(deg * 1e+07) (GnssLat)
  std::int32_t latitude;
  /// @brief Longitude(deg * 1e+07) (GnssLon)
  std::int32_t longitude;
  /// @brief Altitude(m) (GnssAlt)
  float altitude;
  /// @brief Ellipsoid(m) (GnssEllipsoid)
  float ellipsoid;
  /// @brief PoseType (PoseType)
  PoseType pose_type;
  /// @brief Position status (GnssPosStatus)
  PositionStatus position_status;
  /// @brief Num satellites (SatsNum)
  std::uint8_t num_satellites;
  /// @brief Padding
  std::uint8_t padding;
  /// @brief GnssVelNorth(m/s)
  float gnss_vel_north;
  /// @brief GnssVelEast(m/s)
  float gnss_vel_east;
  /// @brief GnssVelDown(m/s)
  float gnss_vel_down;
  /// @brief Heading(deg) (GnssHeading)
  float heading;
  /// @brief Course(deg) (GnssCourse)
  float course;
  /// @brief Heading error(deg) (GnssHeadingErr)
  float heading_error;
  /// @brief Course error(deg) (GnssCourseErr)
  float course_error;
  /// @brief Horizon position error(m) (GnssHoriPosErr)
  float horizon_position_error;
  /// @brief Vertical position error(m) (GnssVertPosErr)
  float vertical_position_error;
  /// @brief Horizon velocity error(m/s) (GnssHoriVelErr)
  float horizon_velocity_error;
  /// @brief Vertical velocity error(m/s) (GnssVertVelErr)
  float vertical_velocity_error;
  /// @brief T DOP (GnssTdop)
  float tdop;
  /// @brief H DOP (GnssHdop)
  float hdop;
  /// @brief V DOP (GnssVdop)
  float vdop;
  /// @brief Num gnss satellite info
  std::uint8_t num_gnss_satellite_info;
  /// @brief Satellite sequence number
  std::uint16_t sat_seq_no;
  /// @brief padding
  std::uint8_t padding_1;
  /// @brief Satellite info
  SatelliteInfo statelline_info[kMaxSatInfoNum];
};

struct alignas(8) GnssComm {
  /// @brief high time stamp
  std::uint32_t high_timestamp;
  /// @brief high time stamp
  std::uint32_t low_timestamp;
  /// @brief frame id
  std::uint32_t frame_id;
  /// @brief Padding
  std::uint8_t padding[4];
  /// @brief Gnss data
  Gnss gnss;
};

}
}


#endif