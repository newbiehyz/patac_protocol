//
#ifndef AUTODRIVE_TYPE_DYNAMIC_OBSTACLE_DEBUG_INFO_H_
#define AUTODRIVE_TYPE_DYNAMIC_OBSTACLE_DEBUG_INFO_H_

#include <memory>

#include "perception_base.h"

namespace autodrive {

    static constexpr std::uint32_t kPMS_ENUM_SIZE{16U}; // equal to PMS_ENUM_SIZE
    static constexpr std::uint32_t kSpeedMeasureCnt{4U};

    /// @brief position result by each method
    struct alignas(8) PositionResult
    {
        /// @brief position
        Vector3f position;
        /// @brief variation
        Vector3f var;
        ///@brief method type(PositionMeasuremtSource)
        std::uint16_t type;
        std::uint8_t padding[6];
};

/// @brief position debug info
struct alignas(8) PositionDebugInfo
{
    /// @brief the obj in which sensors(binary 1 means in this sensor)
    std::int32_t in_sensors;
    /// @brief use rear or head or 3d (binary 1 means use this measure)
    std::int32_t use_measures;
    /// @brief position result
    std::array<PositionResult, kPMS_ENUM_SIZE> position_results;
    /// @brief number of position_results
    std::int32_t num_position_results;
    /// @brief last speed result in KF
    float speed;
    /// @brief position result we use
    PositionResult fusion_pos;
    /// @brief last position in KF
    PositionResult pre_pos;
    /// @brief KF result position
    PositionResult post_pos;
    /// @brief time gap of last state and this state
    std::int64_t time_gap;
};

/// @brief speed measure info
struct alignas(8) SpeedResult
{
/// @brief HScaleSpeed = 0, Real3DScaleSpeed = 1
    // WScaleSpeed = 2, BPointScaleSpeed = 3
    std::uint16_t speed_type;
    /// @brief use rear or full rect, use left or right side
    std::uint16_t measure_type;
    /// @brief masure valid
    std::uint8_t measure_isvalid;
    std::uint8_t padding[3];
    /// @brief now it is H
    float real_size;
    /// @brief reference point image x
    float img_x;
    /// @brief measure h
    float h;
    /// @brief measure w
    float w;
    /// @brief position in camera or vcs
    Vector2f raw_position; 
    /// @brief position in world
    Vector2f world_position; 
    /// @brief position noise
    Vector2f pos_noise;

    /// @brief state before KF
    Vector2f state_pos; //position
    Vector2f state_pos_noise; /// position noise
    Vector2f state_speed; // speed 
    Vector2f state_speed_noise; //speed noise
};

struct alignas(8) SpeedDebugInfo
{
    /// @brief speed result
    std::array<SpeedResult, kSpeedMeasureCnt> speed_results;
    /// @brief number of speed_results
    std::int32_t num_speed_results;
    std::uint8_t padding[4];
    /// @brief speed result
    Vector2f speed;
    /// @brief speed noise
    Vector2f speed_noise;
    /// @brief acceleration
    Vector2f acc;
    /// @brief acceleration noise
    Vector2f acc_noise;
};

/// @brief obj size debug info
struct alignas(8) SizeDebugInfo
{
    /// @brief   VEH_TYPE_UNKOWN = 0,
    //   VEH_TYPE_BUS = 1,
    //   VEH_TYPE_CAR = 2,
    //   VEH_TYPE_TRUCK = 3,
    //   VEH_TYPE_MOTORS = 4,
    //   VEH_TYPE_SPEC_VEH = 5,
    //   VEH_TYPE_TINY_CAR = 6,
    //   VEH_TYPE_LORRY = 7,
    //   VEH_TYPE_MINIVAN = 8
    std::uint16_t veh_type;
    std::uint8_t padding[6];
    /// @brief type confidence
    float veh_type_conf;
    /// @brief raw H measure
    float H_measure_raw;
    /// @brief H measure
    float H_measure;
    /// @brief H noise
    float H_measure_noise;

    /// @brief H from WH scale
    float H_wh_measure_raw;
    /// @brief its measure
    float H_wh_measure;
    /// @brief its noise
    float H_wh_measure_noise;

    /// @brief H from 3D
    float H_3d_measure;
    /// @brief H noise
    float H_3d_measure_noise;
    /// @brief W from 3D
    float W_3d_measure;
    /// @brief W noise
    float W_3d_measure_noise;
    /// @brief L from 3D
    float L_3d_measure;
    /// @brief L noise
    float L_3d_measure_noise;

    /// @brief L H W result
    float W;
    float H;
    float L;
};

/// @brief yaw debug info
struct alignas(8) YawDebugInfo
{
    /// @brief yaw measure type
    //  YAW_RECT = 0,
    //  YAW_REAL3D = 1,
    //  YAW_BORDER_SIDE = 2,
    //  YAW_PARALLEL = 3,
    //  YAW_FREESPACE = 4,
    //  YAW_IMG_DIST2LANE = 5,
    //  YAW_WHEEL = 6,
    //  YAW_IPM_PARSING = 7
    std::uint16_t yaw_type;
    std::uint8_t padding[2];
    /// @brief yaw measure in vcs
    float vcs_pi_measure_yaw;
    /// @brief yaw measure in world
    float world_pi_measure_yaw;
    /// @brief yaw measure variation
    float yaw_measure_var;

    /// @brief yaw state in vcs
    float vcs_pi_yaw;
    /// @brief yaw state in world
    float world_pi_yaw;
    /// @brief yaw rate state
    float yaw_rate;
    /// @brief yaw state variation
    float yaw_var;
};

/// @brief vision object debug info for developer
struct alignas(8) DynamicObstacleDebugInfo
{
    /// @brief obj id
    std::int32_t id;
    /// @brief frame id
    std::int32_t frame_id;
    /// @brief Camera index
    std::int32_t sensor_id;
    std::uint8_t padding[4];
    /// @brief position debug info
    PositionDebugInfo pos_debug_info;
    /// @brief speed debug info
    SpeedDebugInfo speed_debug_info;
    /// @brief object size debug info
    SizeDebugInfo size_debug_info;
    /// @brief yaw debug info
    YawDebugInfo yaw_debug_info;
};

typedef std::shared_ptr<DynamicObstacleDebugInfo> spDynamicObstacleDebugInfo;

void ResetPositionResult(PositionResult &pos_res);

void ResetPositionDebugInfo(PositionDebugInfo &pos_debug_info);

void ResetSpeedResult(SpeedResult &speed_res);

void ResetSpeedDebugInfo(SpeedDebugInfo & speed_debug_info);

void ResetSizeDebugInfo(SizeDebugInfo& size_debug_info);

void ResetYawDebugInfo(YawDebugInfo& yaw_debug_info);

void ResetObstacleDebugInfo(DynamicObstacleDebugInfo &obstacle_debug_info);

}// namespace autodrive

#endif//AUTODRIVE_TYPE_DYNAMIC_OBSTACLE_DEBUG_INFO_H_