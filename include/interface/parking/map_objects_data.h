//
#ifndef AUTODRIVE_INTERFACE_OBJECTS_DATA_H_
#define AUTODRIVE_INTERFACE_OBJECTS_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>
#include "type/perception_base.h"

namespace autodrive {

class MapObjectsData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxStrIdSize{50U};
  static constexpr std::uint32_t kMaxObjectSize{50U};
  static constexpr std::uint32_t kMaxObjectAttrValueSize{10U};

  enum class MapObjectType : std::uint8_t {
    kUnknown = 0,
    kTrafficSign = 1,
    kTrafficLight = 2,
    kTrafficLightBulb = 3,
    kLaneMarking = 4,
    kStopLine = 5,
    kSpeedBump = 6,
    kPole = 7,
    kCrossWalk = 8,
    kZone = 9,
    kParkingSlot = 10,
    kTrafficCone = 11,
    kParkingLockClose = 12,
    kParkingLockOpen = 13,
    kParkingColumn = 14,
    kParkingAFrameSign = 15,
    kJunction = 16,
    kParkingLock = 17,
    /// @brief Connected to the wall
    kCementColumn_Yes = 18,
    /// @brief Not connected to the wall
    kCementColumn_No = 19
  };

  struct alignas(8) MapObjectAttr {
    /// @brief Object shape
    int32_t shape;
    /// @brief Cycle time in s
    float cycle_time;
    /// @brief Traversable
    int32_t traversable;
    /// @brief Height
    float height;
    /// @brief Radius
    float radius;
    /// @brief Object status
    int32_t status;
    std::array<char, kMaxObjectAttrValueSize> value;
    std::uint8_t num_value;
    /// @brief Is dynamic?
    bool dynamic;
    std::uint8_t padding[4];
    /// @brief Image info
    ImageSpace img_info;
  };

  enum class MapObjectSubType : std::uint8_t {
    kParkLockTypeUnknown = 0,
    kParkLockTypeOpen = 1,
    kParkLockTypeClose = 2,
    kFireHydrantTypeAir = 3,
    kFireHydrantTypeGnd = 4
  };

  enum class MapObjectShape : std::uint8_t {
    kUnknown = 0,
    kRectangle = 1,
    kTriangle = 2,
    kRound = 3,
    kCylinder = 4
  };

  struct alignas(8) MapObject {
    /// @brief Object id
    std::int32_t id;
    /// @brief Camera id
    std::int32_t cam_id;
    /// @brief Time stamp
    std::int64_t time_stamp;
    /// @brief Confidence 0-100
    std::int32_t conf;
    /// @brief Parking slots, collumn...
    MapObjectType type;
    /// @brief Sub type of the object, e.g.: open?closed?
    MapObjectSubType sub_type;
    std::uint8_t padding_1[2];
    /// @brief Border of the object
    Polygon border;
    /// @brief Life time in ms
    std::int32_t life_time;
    /// @brief Frame count
    std::int32_t age;
    /// @brief Globl id ,uuid
    std::array<char, kMaxStrIdSize> str_id;
    std::uint8_t num_str_id;
    std::uint8_t padding_2[5];
    /// @brief Object attributes
    MapObjectAttr attr;
    /// @brief Position in vcs
    Vector3f position;
  };

  struct alignas(8) MapObjects {
    /// @brief Confidence value scale factor
    float conf_scale;
    /// @brief Number of objects
    std::int32_t num_objects;
    /// @brief Objects
    std::array<MapObject, kMaxObjectSize> objects;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit MapObjectsData(std::int64_t timestamp) noexcept;

  MapObjectsData() noexcept;
  ~MapObjectsData() noexcept final = default;
  MapObjectsData(MapObjectsData const&) = default;
  MapObjectsData& operator=(MapObjectsData const&) = default;
  MapObjectsData(MapObjectsData&&) = default;
  MapObjectsData& operator=(MapObjectsData&&) = default;

  /// @brief Get Objects data
  ///
  /// @return objects_
  MapObjects const& GetMapObjects() const noexcept;

  /// @brief Mutable Objects data
  ///
  /// @return objects_
  MapObjects& MutableMapObjects() noexcept;

  /// @brief Set Objects data
  ///
  /// @param objects_data objects_ data
  void SetMapObjects(MapObjects const& objects_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  MapObjects objects_;
};

}  // namespace autodrive

#endif
