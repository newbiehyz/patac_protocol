//
#ifndef AUTODRIVE_INTERFACE_REFERENCE_LINES_DATA_H_
#define AUTODRIVE_INTERFACE_REFERENCE_LINES_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "map_display_data.h"
#include "odometry_data.h"
#include "type/loc_odo.h"
#include "type/ndm_base.h"

namespace autodrive {

class ReferenceLinesData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxWayPointsSize{10000U};
  static constexpr std::uint32_t kMaxStrObjIdSize{64U};
  static constexpr std::uint32_t kMaxReferenceLineHeaderSize{64U};
  static constexpr std::uint32_t kMaxReferencePointsSize{500U};

  // @brief Definition of RefLineWayPointGlobal struct for global planning path
  // points
  struct alignas(8) RefLineWayPointGlobal {
    /// @brief 3D position (x, y , z)
    Vector3f waypoint_position;
    /// @brief Yaw angle
    float waypoint_yaw;
    /// @brief Pitch angle
    float waypoint_pitch;
    /// @brief Roll angle
    float waypoint_roll;
    /// @brief Curvature
    float waypoint_curv;
    /// @brief Rate of change of curvature
    float waypoint_curvrate;
    /// @brief Speed
    float waypoint_speed;

    /// @brief Number of path points
    uint32_t waypoint_num;
    /// Distance from starting point
    float waypoint_dist;

    enum class WaypointDir : std::uint8_t {
      kForward = 0,
      kBackward = 1,
    };
    /// @brief Direction
    WaypointDir waypoint_dir;
    /// @brief Whether the path point can be changed, not used
    bool waypoint_locked;
    /// @brief Is map origin point
    bool is_map_origin;
    /// @brief Is intersection
    bool is_intersection;
    /// @brief Right Turn light status
    OdometryData::TurningLightStatus left_turning_light_status;   
    /// @brief Left Turn light status
    OdometryData::TurningLightStatus right_turning_light_status;
    /// @brief Is parking gate
    bool is_parking_gate;
    /// @brief Is speed bump
    bool is_speed_bump;

    /// @brief way point floor
    float waypoint_floor;
    /// @brief Left Bound Distance
    float left_bound_distance;

    /// @brief Right Bound Distance    
    float right_bound_distance;
     std::uint8_t padding[4];
  };

  /// @brief Definition of RefLineWayPointFrameGlobal struct for global planning
  /// path points frame
  struct alignas(8) RefLineWayPointFrameGlobal {
    /// @brief ID of the global trajectory for matching with map data
    uint32_t waypoint_id;
    /// @brief Number of planning path points
    uint32_t num_waypoints;
    /// Planning path points
    std::array<RefLineWayPointGlobal, kMaxWayPointsSize> way_points;
  };

  /// @brief Definition of ReferenceLinePoint struct
  struct alignas(8) ReferenceLinePoint {
    /// @brief Enumeration of vertex types
    enum class VertexType : std::uint8_t {
      KUnknown = 0,
      KNoVertex = 1,
      KVertexDist = 2,
      KVertexYaw = 3,
      KVertexPitch = 4
    };

    /// @brief Odometry
    Odometry odo;
    /// @brief Pitch angle
    float pitch;
    /// @brief Number of satellites
    int32_t num_sates;
    /// @brief Floor number of corresponding node
    int32_t floor_num;
    /// @brief Vertex type
    VertexType vertex_type;
    /// @brief Whether the corresponding reference point is within map range
    bool has_map;
    /// @brief Flag for initialization location region
    bool init_loc_region;
    uint8_t num_str_obj_id;
    /// @brief UUIDs of associated objects nearby
    std::array<char, kMaxStrObjIdSize> str_obj_id;
  };

  /// @brief Definition of RefLine struct
  struct alignas(8) RefLine {
    /// @brief Number of header
    uint8_t num_header;
    /// @brief padding
    std::uint8_t padding[7];
    /// @brief Number of ref points
    std::uint32_t num_reference_points;
    /// @brief Floor number of the points
    int32_t map_floor_num;
    /// @brief Header of ref point, char
    std::array<char, kMaxReferenceLineHeaderSize> header;
    /// @brief Reference points
    std::array<ReferenceLinePoint, kMaxReferencePointsSize> reference_points;
  };

  /// @brief Definition of RefLineLayer struct
  struct alignas(8) ReferenceLines {
    /// @brief Before optimization
    RefLineWayPointFrameGlobal waypoint_frame_global_raw;
    /// @brief After optimization
    RefLineWayPointFrameGlobal waypoint_frame_global;
    /// @brief Final selected parking slot object
    MapDisplayData::GlbPrkgSlot parking_slot;
    /// @brief reference trajectory for initialization location
    RefLine reference_line;
    /// @brief Ndm header
    NdmSpec ndm_header;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ReferenceLinesData(std::int64_t timestamp) noexcept;

  ReferenceLinesData() noexcept;
  ~ReferenceLinesData() noexcept final = default;
  ReferenceLinesData(ReferenceLinesData const &) = default;
  ReferenceLinesData &operator=(ReferenceLinesData const &) = default;
  ReferenceLinesData(ReferenceLinesData &&) = default;
  ReferenceLinesData &operator=(ReferenceLinesData &&) = default;

  /// @brief Get Reference Lines data
  ///
  /// @return reference_lines_
  ReferenceLines const &GetReferenceLines() const noexcept;

  /// @brief Mutable Reference Lines data
  ///
  /// @return reference_lines_
  ReferenceLines &MutableReferenceLines() noexcept;

  /// @brief Set Reference Lines data
  ///
  /// @param reference_lines_data reference lines data
  void SetReferenceLines(ReferenceLines const &reference_lines_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  ReferenceLines reference_lines_;
};

}  // namespace autodrive

#endif
