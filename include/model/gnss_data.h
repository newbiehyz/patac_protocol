//
#ifndef AUTODRIVE_MODEL_GNSS_DATA_H_
#define AUTODRIVE_MODEL_GNSS_DATA_H_

#include <string.h>
#include <cstdint>
#include <memory>
#include <array>
#include <foundation/type/common_msg_data.h>
#include "model/gnss_comm.h"

namespace autodrive {

// Gnss data
class GnssData final : public CommonMsgData {
 public:
  /// @brief Max satellite info number
  static constexpr std::uint8_t kMaxSatInfoNum{36U};

  // Position status
  enum class PositionStatus : std::uint8_t {
    INVALID = 0,         // Invalid
    DR = 1,              // DR
    FIXED_2D = 2,        // 2D fixed
    FIXED_3D = 3,        // 3D fixed
    GNSS_DR = 4,         // GNSS + DR
    ONLY_TIME_VALID = 5  // Only time valid
  };

  // Positing system
  enum class PositingSystem : std::uint8_t {
    GPS = 0,      // GPS
    BEI_DOU = 1,  // Bei dou
    GLONASS = 2,  // Glonass
    GALILEO = 3,  // Galileo
    QZSS = 4,     // Qzss
    SBAS = 5,     // Sbas
    UNKNOWN = 6   // Unknown
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
    INVALID = 0,        // Invalid
    SINGLE = 1,         // Single
    DGPS = 2,           // DGPS
    INVALID_PPS = 3,    // Invalid pps
    RTK_FIXED = 4,      // Rtk fixed
    RTK_FLOAT = 5,      // Rtk float
    DEAD_RECKONING = 6  // Dead reckoning
  };

  // GNSS
  struct alignas(8) Gnss {
    /// @brief GNSS time (GNSS_TimeStamp)
    std::uint64_t utc_time;
    /// @brief Latitude(deg * 1e+07) (GnssLat)
    std::int32_t latitude;
    /// @brief Longitude(deg * 1e+07) (GnssLon)
    std::int32_t longitude;
    /// @brief Altitude(m) (GnssAlt)
    float altitude;
    /// @brief Ellipsoid(m) (GnssEllipsoid)
    float ellipsoid;
    /// @brief Quality (GnssQuality)
    PoseType pose_type;
    /// @brief Position status (GnssPosStatus)
    PositionStatus position_status;
    /// @brief Num satellites (SatsNum)
    std::uint8_t num_satellites;
    /// @brief padding
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
    /// @brief Satellite sequence number
    std::uint16_t sat_seq_no;
    /// @brief Num gnss satellite info
    std::uint8_t num_gnss_satellite_info;
    /// @brief padding
    std::uint8_t padding_1;
    /// @brief Satellite info
    std::array<SatelliteInfo, kMaxSatInfoNum> statelline_info;
  };

  GnssData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit GnssData(std::int64_t timestamp) noexcept;
  ~GnssData() noexcept final = default;
  GnssData(GnssData const&) = default;
  GnssData& operator=(GnssData const&) = default;
  GnssData(GnssData&&) = default;
  GnssData& operator=(GnssData&&) = default;

  /// @brief Get gnss
  ///
  /// @return gnss_
  Gnss const& GetGnss() const noexcept;

  /// @brief Get mutable gnss
  ///
  /// @return gnss_
  Gnss& MutableGnss() noexcept;

  /// @brief Set gnss
  ///
  /// @param Gnss  gnss_
  void SetGnss(Gnss const& gnss) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

  /// @brief fill data
  ///
  /// @param comm_ptr gnss comm ptr
  void FillData(gnss_comm::GnssComm* comm_ptr) noexcept(false);

  /// @brief Fill communication data buffer
  ///
  /// @param data_buf  allocated data buffer
  /// @param nbytes  max length to fill
  ///
  /// @return data length
  std::uint32_t FillCommData(void* data_buf, std::uint32_t nbytes) noexcept(false);

 private:
  /// @brief Gnss
  Gnss gnss_;
};

}

#endif