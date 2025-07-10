//
#ifndef AUTODRIVE_INTERFACE_HMI_MAP_DISPLAY_DATA_H_
#define AUTODRIVE_INTERFACE_HMI_MAP_DISPLAY_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/basic.h"

namespace autodrive {

class HmiMapDisplayData final : public CommonMsgData {
 public:
  using OdomHeading = float;
  using Id16bit = std::uint16_t;

  static constexpr std::uint8_t kMaxBorderSize{4U};
  static constexpr std::uint8_t kMaxMapSize{10U};
  static constexpr std::uint32_t kMaxGlbTrajectoryPointSeNSize{10000U};
  static constexpr std::uint32_t kMaxslotPointsSeNSize{4U};
  static constexpr std::uint32_t kMaxGlbPrkgSlotsSeNSize{1024U};
  static constexpr std::uint32_t kMaxGlbRoadMakMapSeNSize{1000U};
  static constexpr std::uint32_t kMaxGlbPrcpObstacleSeNSize{1000U};

  enum class SlotType : std::uint8_t {
    kReserve = 0,
    kVertical = 1,
    kHorizontal = 2,
    kOblique = 3
  };

  enum class SlotStatus : std::uint8_t {
    /// @brief 非占据（可用）
    kEmpy = 0,
    /// @brief 未知
    kUnknown = 1,
    /// @brief 行人占据
    kPedestrianOcupy = 2,
    /// @brief 两轮车占据
    kTwowheelscartOcupy = 3,
    /// @brief 锥桶占据
    kConebarrelOcupy = 4,
    /// @brief 地锁占据
    kFloorlockOcupy = 5,
    /// @brief 禁停牌占据
    kNoparksignOcupy = 6,
    /// @brief 被其他物体
    kOtherObstaclesOcupy = 7
  };

  enum class MapDataType : std::uint8_t {
    /// @brief The real time map info when mapping
    kRealTimeMap = 0,
    /// @brief Map info loaded from disk
    kStoredMap = 1,
  };

  enum class DangerLevel : std::uint8_t {
    kNoDanger = 0,
    kLevel1 = 1,
    kLevel2 = 2,
    kLevel3 = 3
  };

  enum class Map2HmiObstacleType : std::uint8_t {
    kWheelStopper = 1,
    kParkingLock = 2,
    kPillars = 3,
    kSpeedBump = 4,
    kFireBox = 5,
    kAnticollisionBall = 6,
    kReflector = 7,
    kCurbStone = 8,
    kTrafficCone = 9,
    kParkingAFrameSign = 10,
    kParkingGate = 11  
  };

  enum class RoadMarkType : std::uint8_t {
    kStraight = 2,
    kStrOrTurnL = 3,
    kStrOrTurnR = 4,
    kStrOrTurnA = 5,
    kTurinLeft = 6,
    kTurnLOrTurnA = 7,
    kTurnLOrLConf = 8,
    kTurnR = 9,
    kTurnROrRConf = 10,
    kTurnLR = 11,
    kTurnA = 12,
    kTurnLForb = 13,
    kTurnRForb = 14,
    kTurnAForb = 15,
    kStrOrTurnLOrTrunR = 16,
    kStrOrTurnLOrTrunA = 17,
    kTurnROrTurnA = 18,
    kDistConfLine = 22,
    kStopyieldLine = 23,
    kDecelyieldLine = 24,
    kStopline = 25,
    kNoParkArea = 26,
    kCrossWalk = 27,
    kSolidLaneLine = 28,
    kDashedLaneLine = 29,
    kZebraCrossing = 30,
    kGridLine = 31,
    kLeadingLine = 32,
    kOthers = 99
  };

  enum class ObjStatus : std::uint8_t {
    /// @brief 未知
    kUnknown = 0,
    /// @brief 道闸打开，可通行
    kGateOpen = 1,
    /// @brief 道闸关闭，不可通行
    kGateClose = 2
  };
  struct alignas(8) SingleTrajPoint{
    Vector3f point;
    float floor;
    std::uint8_t padding[4];
  };
  /// @brief global trajectory
  struct alignas(8) GlbTrajectory {
    std::uint8_t padding[4];
    std::uint32_t num_glb_trajectory_point;
    std::array<SingleTrajPoint, kMaxGlbTrajectoryPointSeNSize> glb_trajectory_point;
    Vector3f start_point;
    Vector3f end_point;
  };

  struct alignas(8) GlbPrkgSlot {
    /// @brief slot id
    Id16bit slot_id;
    /// @brief slot status
    SlotStatus slot_status;
    /// @brief slot type
    SlotType slot_type;
    std::uint32_t num_slot_points;
    /// @brief left up; right up; right down; left down
    std::array<Vector3f, kMaxslotPointsSeNSize> slot_points;
    /// @brief has level
    float floor;
    bool has_lever;
    /// @brief is target slot
    bool is_target_slot;
    std::uint8_t padding[2];
    Vector3f lever_start_point;
    Vector3f lever_end_point;
  };

  struct alignas(8) GlbPrcpPrkgSlots {
    std::uint32_t num_glb_prkg_slots;
    std::uint8_t padding[4];
    std::array<GlbPrkgSlot, kMaxGlbPrkgSlotsSeNSize> glb_prkg_slots;
  };

  struct alignas(8) GlbRoadMark {
    float floor;
    /// @brief num of border points
    std::uint8_t num_border_points;
    std::uint8_t padding_1[3];
    /// @brief border points
    std::array<Vector2f, kMaxBorderSize> border;
    /// @brief center points
    Vector3f position;
    OdomHeading odom_heading;
    Id16bit roadmark_id;
    RoadMarkType roadmark_type;
    std::uint8_t padding_2[1];
  };

  struct alignas(8) GlbRoadMakMap {
    std::uint32_t num_glb_road_mak_map;
    std::uint8_t padding[4];
    std::array<GlbRoadMark, kMaxGlbRoadMakMapSeNSize> glb_road_mak_map;
  };

  struct alignas(8) GlbObstacle {
    float floor;
    /// @brief num of border points
    std::uint8_t num_border_points;
    ObjStatus obj_status;
    std::uint8_t padding[2];
    /// @brief border points
    std::array<Vector2f, kMaxBorderSize> border;
    Vector3f center_position;
    OdomHeading odom_heading;
    Map2HmiObstacleType obstacle_type;
    DangerLevel danger_level;
    Id16bit target_id;
  };

  struct alignas(8) GlbPrcpObstacle {
    std::uint32_t num_glb_prcp_obstacle;
    std::uint8_t padding[4];
    std::array<GlbObstacle, kMaxGlbPrcpObstacleSeNSize> glb_prcp_obstacle;
  };

  struct alignas(8) MapCommonHeader {
    /// @brief Id of the map, only valid for MapType::kStoredMap
    std::int32_t map_id;
    /// @brief  Map type
    MapDataType map_type;
    std::uint8_t padding[3];
    /// @brief Timestamp of map
    std::uint64_t utc_time;
    /// @brief Latitude(deg * 1e+07) of map
    std::int32_t latitude;
    /// @brief Longitude(deg * 1e+07) of map
    std::int32_t longitude;
    /// @brief num of global trajectory points
    std::uint32_t num_glb_trajectory_point;
    float map_total_height;
    float map_total_floor;
    /// @brief slot parking times
    std::uint32_t slot_parking_times;
  };

  struct alignas(8) MapInfo {
    /// @brief map common header
    MapCommonHeader header;
    /// @brief global trajectory
    GlbTrajectory global_trajectory;
    /// @brief global percep parking slots
    GlbPrcpPrkgSlots global_percep_parking_slots;
    /// @brief global road mark map
    GlbRoadMakMap global_road_mark_map;
    /// @brief global perception obstacle
    GlbPrcpObstacle global_perception_obstacle;
    /// @brief distance of already mapping (m)
    float map_distance;
    /// @brief rest distance of allow to do mapping (m)
    float rest_map_distance;
    /// @brief num of speed bumps in map
    std::uint8_t num_speed_bumps;
    std::uint8_t padding[7];
  };

  struct alignas(8) MapDisplay {
    /// @brief Map info, only used when update_map_info is true
    MapInfo map_info;
    std::uint8_t padding[6];
    /// @brief Choose update the map info or map headers
    bool update_map_info;
    /// @brief Num of maps, not used when update_map_info is true
    std::int8_t num_maps;
    /// @brief All map headers, not used when update_map_info is true
    std::array<MapCommonHeader, kMaxMapSize> all_map_headers;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit HmiMapDisplayData(std::int64_t timestamp) noexcept;

  HmiMapDisplayData() noexcept;
  ~HmiMapDisplayData() noexcept final = default;
  HmiMapDisplayData(HmiMapDisplayData const&) = default;
  HmiMapDisplayData& operator=(HmiMapDisplayData const&) = default;
  HmiMapDisplayData(HmiMapDisplayData&&) = default;
  HmiMapDisplayData& operator=(HmiMapDisplayData&&) = default;

  /// @brief Get Map Display Data
  ///
  /// @return map_display_
  MapDisplay const& GetMapDisplayData() const noexcept;

  /// @brief Mutable Sync Map Display Data
  ///
  /// @return map_display_
  MapDisplay& MutableMapDisplayData() noexcept;

  /// @brief Set Sync Map Display Data
  ///
  /// @param map_display sync cMap Display Data
  void SetMapDisplayData(MapDisplay const& map_display) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  MapDisplay map_display_;
};

}  // namespace autodrive

#endif
