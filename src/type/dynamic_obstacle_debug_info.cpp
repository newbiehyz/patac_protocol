//

#include "type/dynamic_obstacle_debug_info.h"

namespace autodrive
{
    void ResetPositionResult(PositionResult &pos_res)
    {
        pos_res.position.x = 0.f;
        pos_res.position.y = 0.f;
        pos_res.position.z = 0.f;

        pos_res.type = 0;

        pos_res.var.x = 0.f;
        pos_res.var.y = 0.f;
        pos_res.var.z = 0.f;
        return;
    };
    void ResetPositionDebugInfo(PositionDebugInfo &pos_debug_info)
    {
        pos_debug_info.num_position_results = 0;
        for (size_t i = 0; i < pos_debug_info.position_results.size(); i++)
        {
            ResetPositionResult(pos_debug_info.position_results[i]);
        }
        ResetPositionResult(pos_debug_info.fusion_pos);
        ResetPositionResult(pos_debug_info.pre_pos);
        ResetPositionResult(pos_debug_info.post_pos);
        pos_debug_info.in_sensors = 0;
        pos_debug_info.speed = 0.f;
        pos_debug_info.time_gap = 0;
        pos_debug_info.use_measures = 0;
    };

    void ResetSpeedResult(SpeedResult &speed_res)
    {
        speed_res.h = 0.f;
        speed_res.w = 0.f;
        speed_res.img_x = 0.f;
        speed_res.measure_isvalid = 0;
        speed_res.measure_type = 0;
        speed_res.pos_noise.x = 0.f;
        speed_res.pos_noise.y = 0.f;
        speed_res.raw_position.x = 0.f;
        speed_res.raw_position.y = 0.f;
        speed_res.real_size = 0.f;
        speed_res.speed_type = 0;
        speed_res.state_pos.x = 0.f;
        speed_res.state_pos.y = 0.f;
        speed_res.state_pos_noise.x = 0.f;
        speed_res.state_pos_noise.y = 0.f;
        speed_res.state_speed.x = 0.f;
        speed_res.state_speed.y = 0.f;
        speed_res.state_pos_noise.x = 0.f;
        speed_res.state_pos_noise.y = 0.f;
        speed_res.raw_position.x = 0.f;
        speed_res.raw_position.y = 0.f;
        speed_res.world_position.x = 0.f;
        speed_res.world_position.y = 0.f;
        return;
    };

    void ResetSpeedDebugInfo(SpeedDebugInfo &speed_debug_info)
    {
        speed_debug_info.num_speed_results = 0;
        for (size_t i = 0; i < speed_debug_info.speed_results.size(); i++)
        {
            ResetSpeedResult(speed_debug_info.speed_results[i]);
        }
        speed_debug_info.acc.x = 0.f;
        speed_debug_info.acc.y = 0.f;
        speed_debug_info.acc_noise.x = 0.f;
        speed_debug_info.acc_noise.y = 0.f;
        speed_debug_info.speed.x = 0.f;
        speed_debug_info.speed.y = 0.f;
        speed_debug_info.speed_noise.x = 0.f;
        speed_debug_info.speed_noise.y = 0.f;
    };

    void ResetSizeDebugInfo(SizeDebugInfo &size_debug_info)
    {
        size_debug_info.H = 0.f;
        size_debug_info.H_3d_measure = 0.f;
        size_debug_info.H_3d_measure_noise = 0.f;
        size_debug_info.H_measure = 0.f;
        size_debug_info.H_measure_noise = 0.f;
        size_debug_info.H_measure_raw = 0.f;
        size_debug_info.H_wh_measure = 0.f;
        size_debug_info.H_wh_measure_noise = 0.f;
        size_debug_info.H_wh_measure_raw = 0.f;
        size_debug_info.L = 0.f;
        size_debug_info.L_3d_measure = 0.f;
        size_debug_info.L_3d_measure_noise = 0.f;
        size_debug_info.veh_type = 0;
        size_debug_info.veh_type_conf = 0.f;
        size_debug_info.W = 0.f;
        size_debug_info.W_3d_measure = 0.f;
        size_debug_info.W_3d_measure_noise = 0.f;

        return;
    }

    void ResetYawDebugInfo(YawDebugInfo &yaw_debug_info)
    {
        yaw_debug_info.vcs_pi_measure_yaw = 0.f;
        yaw_debug_info.vcs_pi_yaw = 0.f;
        yaw_debug_info.world_pi_measure_yaw = 0.f;
        yaw_debug_info.world_pi_yaw = 0.f;
        yaw_debug_info.yaw_measure_var = 0.f;
        yaw_debug_info.yaw_rate = 0.f;
        yaw_debug_info.yaw_type = 0;
        yaw_debug_info.yaw_var = 0.f;

        return;
    };

    void ResetObstacleDebugInfo(DynamicObstacleDebugInfo &obstacle_debug_info)
    {
        obstacle_debug_info.frame_id = 0;
        obstacle_debug_info.id = -1;
        obstacle_debug_info.sensor_id = -1;
        ResetPositionDebugInfo(obstacle_debug_info.pos_debug_info);
        ResetSpeedDebugInfo(obstacle_debug_info.speed_debug_info);
        ResetSizeDebugInfo(obstacle_debug_info.size_debug_info);
        ResetYawDebugInfo(obstacle_debug_info.yaw_debug_info);

        return;
    };
}
