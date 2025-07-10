//
#ifndef AUTODRIVE_INTERFACE_ROAD_INFO_DATA_H_
#define AUTODRIVE_INTERFACE_ROAD_INFO_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/perception_base.h"

namespace autodrive {

class RoadInfoData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxPointsSize{64U};
  static constexpr std::uint32_t kMaxTSize{64U};
  static constexpr std::uint32_t kMaxXCoeffSize{4U};
  static constexpr std::uint32_t kMaxYCoeffSize{4U};
  static constexpr std::uint32_t kMaxBordersSize{8U};
  static constexpr std::uint32_t kMaxStrIdSize{4U};
  static constexpr std::uint32_t kMaxCurveSegsSize{2U};
  static constexpr std::uint32_t kMaxLinesSize{12U};
  static constexpr std::uint32_t kMaxTransitionPointsSize{8U};

  enum class RoadBorderSide : std::uint8_t { kLeftEdge = 0, kRightEdge = 1 };

  enum class RoadBorderType : std::uint8_t {
    kCurb = 0,
    kRoadEdge = 1,
    kBarrier = 2,
    kCone = 3,
    kParkedCars = 4,
    kWall = 5,
    kGuardRail = 6,
    kOther = 7
  };

  /// @brief Confidence level of the line
  enum class RoadInfoLineSource : std::uint8_t {
    kRoadConfNotAvailable = 0,
    kRoadConfPredicted = 1,
    kRoadConfDetected = 2
  };

  enum class RoadInfoCurveType : std::uint8_t {
    kUnknown = 0,
    kAdas = 2,
    kCubic = 4,
    kBezier = 8
  };

  /// @brief
  /// 1. Segmentation points: separation or merging points where there is a
  /// significant change in curvature, or at the inflection points of an S-curve
  /// where the second derivative is zero.
  /// 2.Smooth constraint using the first derivative at the segmentation points.
  struct alignas(8) RoadInfoCurveLine {
    /// @brief The start of the curve line,unit:m;
    float start;
    /// @brief The end of the curve line, unit : m;
    float end;
    /// @brief The length of the curve line, unit : m; same as
    /// measuredViewRange;
    float length;
    /// @brief Fit error of Root mean squared error
    float rmse;
    RoadInfoCurveType type;
    std::uint8_t num_points;
    std::uint8_t num_t;
    std::uint8_t num_x_coeff;
    std::uint8_t num_y_coeff;
    std::uint8_t padding[3];
    ///@brief Indicate the contour point in image;
    std::array<Vector3d, kMaxPointsSize> points;
    std::array<double, kMaxTSize> t;
    double t_max;
    Vector3d start_pt;
    /// @brief if type == CurveType_Cubic,
    /// x = x_coeff[0] + x_coeff[1] * t + x_coeff[2] * t^2 + x_coeff[3] * t^3
    std::array<double, kMaxXCoeffSize> x_coeff;
    /// @brief if type == CurveType_Cubic,
    /// y = y_coeff[0] + y_coeff[1] * t + y_coeff[2] * t^2 + y_coeff[3] * t^3
    /// else if type == CurveType_ADAS,
    /// y = y_coeff[0] + y_coeff[1] * x + y_coeff[2] * x^2 + y_coeff[3] * x^3
    /// 1. The default type is CurveType_ADAS;
    /// 2. The y_coeff represents the polynomial coefficient，which means if you
    /// want get curvature coefficient at x =0，you need multiply by some value;
    /// laterl position       : y_coeff[0], unit:m;
    /// heading angle         : atan(y_coeff[1]), unit:rad;
    /// curvature             : 2*y_coeff[2], unit:1/m;
    /// Curvature change rate : 6*y_coeff[3], unit:1/m^2;s
    std::array<double, kMaxYCoeffSize> y_coeff;
  };

  struct alignas(8) Border {
    RoadBorderSide border_side;
    RoadBorderType border_type;
    /// @brief  Indicate if one border is valid.
    /// value                  detail
    /// kRoadConfNotAvailable : the border is invalid;
    ///    kRoadConfPredicted : the border was from predicting, which means the
    ///    line was not detected in current frame.
    ///     kRoadConfDetected : the border was detected;
    RoadInfoLineSource conf_level;
    std::uint8_t padding[5];
    RoadInfoCurveLine curve;
    float conf;
    /// @brief The height of the road border (-99 if n/a)
    float border_height;
  };

  struct alignas(8) RoadBorders {
    std::int32_t nums_borders;
    std::uint8_t padding[4];
    /// @brief array of border(road edge)
    /// 1.The array size is 2;
    /// 2.The storage order is left border and right border;
    /// the key words conf_level in Border indicate if one border is valid. the
    /// value "kRoadConfNotAvailable" means the border is invalid;
    std::array<Border, kMaxBordersSize> borders;
  };

  enum class RoadInfoLineType : std::uint8_t {
    kUndecided = 0,
    kSolidLine = 1,
    kDashedLine = 2,
    kDoubleLine = 3,
    kDecelerationLine = 4,
    kBotts = 5,
    kHOV = 6,
  };

  enum class DlmType : std::uint8_t {
    kDlmNotDlm = 0,
    kDlmSolidDashed = 1,
    kDlmDashedSolid = 2,
    kDlmSolidSolid = 3,
    kDlmDashedDashed = 4,
    kDlmUndecided = 5
  };

  enum class DecelType : std::uint8_t {
    kDecelNotDecel = 0,
    kDecelSolid = 1,
    kDecelDashed = 2,
    kDecelUndecided = 3,
    kDecelInvalid = 4
  };

  enum class RoadInfoLineColor : std::uint8_t {
    kUnknown = 0,
    kWhite = 1,
    kYellow = 2,
    kBlue = 3,
    kGreen = 4
  };

  enum class RoadInfoLinePosition : std::uint8_t {
    kUnknown = 0,
    kLeft = 1,
    kRight = 2,
    kLeftLeft = 3,
    kRightRight = 4,
    kLeftOutside = 5,   // the left out side branch
                        // for Y structure lines.
    kRightOutside = 6,  // the right out side branch
                        // for Y structure lines
    kLeftRight = 7,
    kRightLeft = 8,
    kLeftLeftLeft = 9,
    kRightRightRight = 10
  };

  enum class TransitionType : std::uint8_t {
    kTransitionTypeSplit = 0,
    kTransitionTypeMerge = 1,
    kTransitionTypeExit = 2
  };

  enum class LineRole : std::uint8_t {
    kUnknown = 0,
    kLeft = 1,
    kRight = 2,
    kLeftLeft = 3,
    kRightRight = 4,
    kLeftEdge = 5,
    kRightEdge = 6
  };

  struct alignas(8) TransitionPoints {
    float long_position;
    float lat_position;
    float prob;
    TransitionType type;
    /// @brief Indicate the related line or border of the transition points;
    LineRole relate_line;
    std::uint8_t padding[2];
  };

  enum class TriggerModel : std::uint8_t {
    kNotAvailable = 0,
    kTriggered = 1,
    kNotTriggered = 2
  };

  struct alignas(8) RoadInfoLine {
    std::int32_t id;
    std::array<char, kMaxStrIdSize> str_id;
    std::uint8_t num_str_id;
    std::uint8_t padding_1[7];
    /// @brief From near to far of the ego vehicle
    std::array<RoadInfoCurveLine, kMaxCurveSegsSize> curve_segs;
    std::int32_t num_curve_segs;
    std::int32_t life_time;
    /// @brief The start of the whole line,which may contain more than one
    /// segment,unit:m;
    float start;
    /// @brief The end of the whole line,unit:m;
    float end;
    /// @brief The length of the whole line,unit:m;same as measuredViewRange;
    float length;
    /// @brief The single frame probability of a line candidate to be a real
    /// line
    float conf;
    /// @brief The width of lane line,unit:m;
    float width;
    RoadInfoLinePosition position;
    /// @brief  Indicate if one line is valid.
    /// value                  detail
    /// kRoadConfNotAvailable : the line is invalid;
    ///    kRoadConfPredicted : the line was from predicting, which means the
    ///    line was not detected in current frame.
    ///     kRoadConfDetected : the line was detected;
    RoadInfoLineSource conf_level;
    RoadInfoLineColor line_color;
    RoadInfoLineType type;
    /// @brief Double line type
    DlmType dlm_type;
    /// @brief Deceleration type
    DecelType decel_type;
    /**
     * Prediction information describing.
     * bit 0 = None
     * bit 1 = Occluded
     * bit 2 = Other_Side
     * bit 3 = Override
     * bit 4 = Dist_Based_Extrapolation
     * bit 5 = Headway_Oriented
     */
    std::uint8_t road_prediction;
    TriggerModel triggered_model;
    std::uint8_t padding[4];
  };

  struct alignas(8) HighWayJunctionInfo {
    bool highway_exit_left;
    bool highway_exit_right;
    bool highway_entry_left;
    bool highway_entry_right;
    std::uint8_t padding[4];
  };

  struct alignas(8) NumberOfLanes {
    std::uint8_t range_0m;
    std::uint8_t range_35m;
    std::uint8_t padding[6];
  };

  enum class DrivingSide : std::uint8_t {
    kLeftSide = 0,
    kRightSide = 1,
  };

  /// @brief Indicates when the vision system detects a lane change.
  /// The member value is set to > 0 for exactly one cycle
  /// when the lane change occurs.
  enum class LaneChange : std::uint8_t {
    kNoChange = 0,
    kChangeToLeft = 1,
    kChangeToRight = 2,
    kChangeToUnknown = 3,
  };

  struct alignas(8) RoadInfoLines {
    /// @brief Camera matrix
    CameraMatrix camera_matrix;
    /// @brief List of lines
    /// 1.The array size is fixed as 4,the output only contain four lines or
    /// three lanes; 2.The output order is host lane and side lane, details are
    /// as fellow: index: lane positon
    ///     0: kleft
    ///     1: kright
    ///     2: kleftleft
    ///     3: krightright
    /// the key words conf_level in RoadInfoLine indicate if one line is valid.
    /// the value "kRoadConfNotAvailable" means the line is invalid; if
    /// ystructure was found at one host line,for example, a branch was found on
    /// the left host line, the output is: index: lane positon
    ///     0: kleft
    ///     1: kright
    ///     2: kLeftOutside
    ///     3: krightright
    /// if left host line, the output is:
    /// index: lane positon
    ///     0: kleft
    ///     1: kright
    ///     2: kLeftLeft
    ///     3: krightOutside
    std::array<RoadInfoLine, kMaxLinesSize> lines;
    std::int32_t num_lines;
    std::uint8_t padding_1[4];
    /// @brief Indicates the output timestamp of the road line;
    std::int64_t src_time_stamp;
    std::int64_t asyn_time_stamp;
    std::uint8_t ambiguous_line_pattern_left;
    std::uint8_t ambiguous_line_pattern_right;
    std::uint8_t construction_area;
    LaneChange lane_change;
    DrivingSide driving_side;
    std::uint8_t padding_2[3];
    std::array<TransitionPoints, kMaxTransitionPointsSize> transition_points;
    std::int32_t num_transition_points;
    /// @brief Indicate the width of host lane,unit: m
    float host_width;
    HighWayJunctionInfo highway_junction_info;
    NumberOfLanes number_of_lanes;
  };

  enum class StopLineStatus : std::uint8_t {
    kUnknown = 0,
    kInImage = 1,
    kPredicted = 2,
  };

  enum class StopLineType : std::uint8_t {
    kStopLine = 0,
    kDashedStopLine = 1,
    kDoubleStopLine = 2,
    kDashedSolidStopLine = 3,
    kSolidDashedStopLine = 4,
    kDoubleDashedStopLine = 5,
    kTriangularStopLine = 6
  };

  enum class StopLineColorType : std::uint8_t {
    kGreenBlue = 0,
    kWhite = 1,
    kYellowOrangeRed = 2,
  };

  struct alignas(8) StopLines {
    /// @brief The lateral distance to the center of the road arrow
    float lateral_distance;
    /// @brief The longitudinal distance to the center of the road arrow
    float longitudinal_distance;
    float angle;
    float confidence;
    std::uint8_t id;
    StopLineStatus stop_line_status;
    StopLineType stop_line_type;
    StopLineColorType stop_line_color;
    // Specify whether the stop line is relevant to the host vehicle.
    bool is_relevant;
    std::uint8_t padding[3];
  };

  enum class CrossingsType : std::uint8_t {
    kRoadSolidCrossing = 0,
    kRoadZebraCrossing = 1,
    kRoadDashedCrossing = 2
  };

  struct alignas(8) Crossings {
    float lateral_position;
    float longitudinal_position;
    float confidence;
    std::uint8_t id;
    CrossingsType crossings_type;
    std::uint8_t padding[2];
  };

  struct alignas(8) RoadInfo {
    RoadBorders road_borders;
    RoadInfoLines lane_lines;
    StopLines stop_lines;
    Crossings crossings;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit RoadInfoData(std::int64_t timestamp) noexcept;

  RoadInfoData() noexcept;
  ~RoadInfoData() noexcept final = default;
  RoadInfoData(RoadInfoData const&) = default;
  RoadInfoData& operator=(RoadInfoData const&) = default;
  RoadInfoData(RoadInfoData&&) = default;
  RoadInfoData& operator=(RoadInfoData&&) = default;

  /// @brief Get Road Info data
  ///
  /// @return road_info_data_
  RoadInfo const& GetRoadInfo() const noexcept;

  /// @brief Mutable Road Info data
  ///
  /// @return road_info_data_
  RoadInfo& MutableRoadInfo() noexcept;

  /// @brief Set Road Info data
  ///
  /// @param road_info_data road info data
  void SetRoadInfo(RoadInfo const& road_info_data) noexcept;

  /// @brief Get camera_id_
  ///
  /// @return camera_id_
  std::int32_t GetCameraId() const noexcept;

  // @brief Set camera_id_
  ///
  /// @param camera_id_
  void SetCameraId(std::int32_t camera_id) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  std::int32_t camera_id_;
  RoadInfo road_info_data_;
};

}  // namespace autodrive

#endif
