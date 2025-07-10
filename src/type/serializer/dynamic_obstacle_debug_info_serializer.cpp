//

#include "utils/exception.h"
#include "type/serializer/dynamic_obstacle_debug_info_serializer.h"

namespace autodrive
{

static void FillPostionProto(workflow::proto::PositionResult *p_pos_proto,
                        const PositionResult &pos)
{
    p_pos_proto->set_type(pos.type);
    p_pos_proto->mutable_position()->set_x(pos.position.x);
    p_pos_proto->mutable_position()->set_y(pos.position.y);
    p_pos_proto->mutable_position()->set_z(pos.position.z);
    p_pos_proto->mutable_var()->set_x(pos.var.x);
    p_pos_proto->mutable_var()->set_y(pos.var.y);
    p_pos_proto->mutable_var()->set_z(pos.var.z);
    return;
}

static void FillPositionDebugInfoProto(
    workflow::proto::PositionDebugInfo *p_position_debug_proto,
    const PositionDebugInfo &position_debug_info)
{
    p_position_debug_proto->set_in_sensors(position_debug_info.in_sensors);
    p_position_debug_proto->set_use_measures(position_debug_info.use_measures);
    p_position_debug_proto->set_speed(position_debug_info.speed);
    p_position_debug_proto->set_time_gap(position_debug_info.time_gap);
    // position part
    workflow::proto::PositionResult *p_fusion_pos_proto =
        p_position_debug_proto->mutable_fusion_pos();
    const PositionResult &fusion_pos = position_debug_info.fusion_pos;
    FillPostionProto(p_fusion_pos_proto, fusion_pos);

    workflow::proto::PositionResult *p_per_pos_proto =
        p_position_debug_proto->mutable_pre_pos();
    const PositionResult &pre_pos = position_debug_info.pre_pos;
    FillPostionProto(p_per_pos_proto, pre_pos);

    workflow::proto::PositionResult *p_post_pos_proto =
        p_position_debug_proto->mutable_post_pos();
    const PositionResult &post_pos = position_debug_info.post_pos;
    FillPostionProto(p_post_pos_proto, post_pos);

    for (size_t i = 0; i < position_debug_info.num_position_results; i++)
    {
        workflow::proto::PositionResult *position_proto =
            p_position_debug_proto->add_position_results();
        FillPostionProto(position_proto, position_debug_info.position_results[i]);
    }
    return;
}

static void FillSpeedResultProto(workflow::proto::SpeedResult *p_speed_res_proto,
                                 const SpeedResult & speed_res)
{
    // common part
    p_speed_res_proto->set_measure_type(speed_res.measure_type);
    p_speed_res_proto->set_speed_type(speed_res.speed_type);
    p_speed_res_proto->set_measure_isvalid(speed_res.measure_isvalid);
    p_speed_res_proto->set_real_size(speed_res.real_size);
    p_speed_res_proto->set_img_x(speed_res.img_x);
    p_speed_res_proto->set_h(speed_res.h);
    p_speed_res_proto->set_w(speed_res.w);

    // vector2f part
    p_speed_res_proto->mutable_raw_position()->set_x(
        speed_res.raw_position.x);
    p_speed_res_proto->mutable_raw_position()->set_y(
        speed_res.raw_position.y);

    p_speed_res_proto->mutable_world_position()->set_x(
        speed_res.world_position.x);
    p_speed_res_proto->mutable_world_position()->set_y(
        speed_res.world_position.y);

    p_speed_res_proto->mutable_pos_noise()->set_x(
        speed_res.pos_noise.x);
    p_speed_res_proto->mutable_pos_noise()->set_y(
        speed_res.pos_noise.y);

    p_speed_res_proto->mutable_state_pos()->set_x(
        speed_res.state_pos.x);
    p_speed_res_proto->mutable_state_pos()->set_y(
        speed_res.state_pos.y);

    p_speed_res_proto->mutable_state_pos_noise()->set_x(
        speed_res.state_pos_noise.x);
    p_speed_res_proto->mutable_state_pos_noise()->set_y(
        speed_res.state_pos_noise.y);

    p_speed_res_proto->mutable_state_speed()->set_x(
        speed_res.state_speed.x);
    p_speed_res_proto->mutable_state_speed()->set_y(
        speed_res.state_speed.y);

    p_speed_res_proto->mutable_state_speed_noise()->set_x(
        speed_res.state_speed_noise.x);
    p_speed_res_proto->mutable_state_speed_noise()->set_y(
        speed_res.state_speed_noise.y);

    return;
}

static void FillSpeedDebugInfoProto(
    workflow::proto::SpeedDebugInfo *p_speed_debug_proto,
    const SpeedDebugInfo &speed_debug_info)
{
    p_speed_debug_proto->mutable_speed()->set_x(
        speed_debug_info.speed.x);
    p_speed_debug_proto->mutable_speed()->set_y(
        speed_debug_info.speed.y);

    p_speed_debug_proto->mutable_speed_noise()->set_x(
        speed_debug_info.speed_noise.x);
    p_speed_debug_proto->mutable_speed_noise()->set_y(
        speed_debug_info.speed_noise.y);

    p_speed_debug_proto->mutable_acc()->set_x(
        speed_debug_info.acc.x);
    p_speed_debug_proto->mutable_acc()->set_y(
        speed_debug_info.acc.y);

    p_speed_debug_proto->mutable_acc_noise()->set_x(
        speed_debug_info.acc_noise.x);
    p_speed_debug_proto->mutable_acc_noise()->set_y(
        speed_debug_info.acc_noise.y);

    for (size_t i = 0; i < speed_debug_info.num_speed_results; i++)
    {
        workflow::proto::SpeedResult *p_speed_res =
            p_speed_debug_proto->add_speed_results();
        FillSpeedResultProto(p_speed_res, speed_debug_info.speed_results[i]);
    }
    return;
}

static void FillSizeDebugInfoProto(
    workflow::proto::SizeDebugInfo *p_size_debug_proto,
    const SizeDebugInfo &size_debug_info)
{
    p_size_debug_proto->set_veh_type(size_debug_info.veh_type);
    p_size_debug_proto->set_veh_type_conf(size_debug_info.veh_type_conf);

    p_size_debug_proto->set_h_measure_raw(size_debug_info.H_measure_raw);
    p_size_debug_proto->set_h_measure(size_debug_info.H_measure);
    p_size_debug_proto->set_h_measure_noise(size_debug_info.H_measure_noise);

    p_size_debug_proto->set_h_wh_measure_raw(size_debug_info.H_wh_measure_raw);
    p_size_debug_proto->set_h_wh_measure(size_debug_info.H_wh_measure);
    p_size_debug_proto->set_h_wh_measure_noise(size_debug_info.H_wh_measure_noise);

    p_size_debug_proto->set_h_3d_measure(size_debug_info.H_3d_measure);
    p_size_debug_proto->set_h_3d_measure_noise(size_debug_info.H_3d_measure_noise);
    p_size_debug_proto->set_w_3d_measure(size_debug_info.W_3d_measure);
    p_size_debug_proto->set_w_3d_measure_noise(size_debug_info.W_3d_measure_noise);
    p_size_debug_proto->set_l_3d_measure(size_debug_info.L_3d_measure);
    p_size_debug_proto->set_l_3d_measure_noise(size_debug_info.L_3d_measure_noise);

    p_size_debug_proto->set_h(size_debug_info.H);
    p_size_debug_proto->set_w(size_debug_info.W);
    p_size_debug_proto->set_l(size_debug_info.L);

    return;
}

static void FillYawDebugProto(
    workflow::proto::YawDebugInfo *p_yaw_debug_proto,
    const YawDebugInfo &yaw_debug_info)
{
    p_yaw_debug_proto->set_yaw_type(yaw_debug_info.yaw_type);
    p_yaw_debug_proto->set_vcs_pi_measure_yaw(
        yaw_debug_info.vcs_pi_measure_yaw);
    p_yaw_debug_proto->set_world_pi_measure_yaw(
        yaw_debug_info.world_pi_measure_yaw);
    p_yaw_debug_proto->set_yaw_measure_var(
        yaw_debug_info.yaw_measure_var);
    p_yaw_debug_proto->set_vcs_pi_yaw(yaw_debug_info.vcs_pi_yaw);
    p_yaw_debug_proto->set_world_pi_yaw(yaw_debug_info.world_pi_yaw);
    p_yaw_debug_proto->set_yaw_rate(yaw_debug_info.yaw_rate);
    p_yaw_debug_proto->set_yaw_var(yaw_debug_info.yaw_var);

    return;
}

static void FillPositionResult(
    PositionResult &position_result,
    const workflow::proto::PositionResult &position_proto)
{
    position_result.position.x = position_proto.position().x();
    position_result.position.y = position_proto.position().y();
    position_result.position.z = position_proto.position().z();
    position_result.var.x = position_proto.var().x();
    position_result.var.y = position_proto.var().y();
    position_result.var.z = position_proto.var().z();

    position_result.type = position_proto.type();

    return ;
}

static void FillPositionDebugInfo(
    PositionDebugInfo &position_debug_info,
    const workflow::proto::PositionDebugInfo &position_debug_proto)
{
    position_debug_info.in_sensors = position_debug_proto.in_sensors();
    position_debug_info.use_measures = position_debug_proto.use_measures();
    position_debug_info.time_gap = position_debug_proto.time_gap();
    position_debug_info.speed = position_debug_proto.speed();

    if (position_debug_proto.has_fusion_pos())
    {
        PositionResult &fusion_pos = position_debug_info.fusion_pos;
        const workflow::proto::PositionResult &fusion_pos_proto =
            position_debug_proto.fusion_pos();
        FillPositionResult(fusion_pos, fusion_pos_proto);
    }

    if (position_debug_proto.has_pre_pos())
    {
        PositionResult &pre_pos = position_debug_info.pre_pos;
        const workflow::proto::PositionResult &pre_pos_proto =
            position_debug_proto.pre_pos();
        FillPositionResult(pre_pos, pre_pos_proto);
    }

    if (position_debug_proto.has_post_pos())
    {
        PositionResult &post_pos = position_debug_info.post_pos;
        const workflow::proto::PositionResult &post_pos_proto =
            position_debug_proto.post_pos();
        FillPositionResult(post_pos, post_pos_proto);
    }

    for (size_t i = 0;
         i < position_debug_proto.position_results_size() &&
         i < kPMS_ENUM_SIZE;
         i++)
    {
        FillPositionResult(
            position_debug_info.position_results[i],
            position_debug_proto.position_results(i));
    }
    position_debug_info.num_position_results = std::min(
        position_debug_proto.position_results_size(),
        static_cast<int>(kPMS_ENUM_SIZE));

    return;
}

static void FillSpeedResult(
    SpeedResult &speed_result,
    const workflow::proto::SpeedResult & speed_result_proto)
{
    speed_result.speed_type = speed_result_proto.speed_type();
    speed_result.measure_type = speed_result_proto.measure_type();
    speed_result.measure_isvalid = speed_result_proto.measure_isvalid();
    speed_result.real_size = speed_result_proto.real_size();
    speed_result.img_x = speed_result_proto.img_x();
    speed_result.h = speed_result_proto.h();
    speed_result.w = speed_result_proto.w();

    speed_result.raw_position.x = speed_result_proto.raw_position().x();
    speed_result.raw_position.y = speed_result_proto.raw_position().y();

    speed_result.world_position.x = speed_result_proto.world_position().x();
    speed_result.world_position.y = speed_result_proto.world_position().y();

    speed_result.pos_noise.x = speed_result_proto.pos_noise().x();
    speed_result.pos_noise.y = speed_result_proto.pos_noise().y();

    speed_result.state_pos.x = speed_result_proto.state_pos().x();
    speed_result.state_pos.y = speed_result_proto.state_pos().y();

    speed_result.state_pos_noise.x = speed_result_proto.state_pos_noise().x();
    speed_result.state_pos_noise.y = speed_result_proto.state_pos_noise().y();

    speed_result.state_speed.x = speed_result_proto.state_speed().x();
    speed_result.state_speed.y = speed_result_proto.state_speed().y();

    speed_result.state_speed_noise.x = speed_result_proto.state_speed_noise().x();
    speed_result.state_speed_noise.y = speed_result_proto.state_speed_noise().y();

    return;
}

static void FillSpeedDebugInfo(
    SpeedDebugInfo &speed_debug_info,
    const workflow::proto::SpeedDebugInfo &speed_debug_proto)
{
    speed_debug_info.speed.x = speed_debug_proto.speed().x();
    speed_debug_info.speed.y = speed_debug_proto.speed().y();

    speed_debug_info.speed_noise.x = speed_debug_proto.speed_noise().x();
    speed_debug_info.speed_noise.y = speed_debug_proto.speed_noise().y();

    speed_debug_info.acc.x = speed_debug_proto.acc().x();
    speed_debug_info.acc.y = speed_debug_proto.acc().y();

    speed_debug_info.acc_noise.x = speed_debug_proto.acc_noise().x();
    speed_debug_info.acc_noise.y = speed_debug_proto.acc_noise().y();

    for (size_t i = 0; 
        i < speed_debug_proto.speed_results_size() &&
        i < kSpeedMeasureCnt; 
        i++)
    {
        FillSpeedResult(speed_debug_info.speed_results[i],
                        speed_debug_proto.speed_results(i));
    }
    speed_debug_info.num_speed_results =
        std::min(speed_debug_proto.speed_results_size(),
                 static_cast<int>(kSpeedMeasureCnt));

    return;
}

static void FillSizeDebugInfo(
    SizeDebugInfo &size_debug_info,
    const workflow::proto::SizeDebugInfo &size_debug_proto)
{
    size_debug_info.veh_type = size_debug_proto.veh_type();
    size_debug_info.veh_type_conf = size_debug_proto.veh_type_conf();

    size_debug_info.H_measure_raw = size_debug_proto.h_measure_raw();
    size_debug_info.H_measure = size_debug_proto.h_measure();
    size_debug_info.H_measure_noise = size_debug_proto.h_measure_noise();

    size_debug_info.H_wh_measure_raw = size_debug_proto.h_wh_measure_raw();
    size_debug_info.H_wh_measure = size_debug_proto.h_wh_measure();
    size_debug_info.H_wh_measure_noise = size_debug_proto.h_wh_measure_noise();

    size_debug_info.H_3d_measure = size_debug_proto.h_3d_measure();
    size_debug_info.H_3d_measure_noise = size_debug_proto.h_3d_measure_noise();

    size_debug_info.W_3d_measure = size_debug_proto.w_3d_measure();
    size_debug_info.W_3d_measure_noise = size_debug_proto.w_3d_measure_noise();

    size_debug_info.L_3d_measure = size_debug_proto.l_3d_measure();
    size_debug_info.L_3d_measure_noise = size_debug_proto.l_3d_measure_noise();

    size_debug_info.H = size_debug_proto.h();
    size_debug_info.W = size_debug_proto.w();
    size_debug_info.L = size_debug_proto.l();

    return;
}

static void FillYawDebugInfo(
    YawDebugInfo &yaw_debug_info,
    const workflow::proto::YawDebugInfo &yaw_debug_proto)
{
    yaw_debug_info.yaw_type = yaw_debug_proto.yaw_type();
    yaw_debug_info.vcs_pi_measure_yaw = yaw_debug_proto.vcs_pi_measure_yaw();
    yaw_debug_info.world_pi_measure_yaw = yaw_debug_proto.world_pi_measure_yaw();
    yaw_debug_info.yaw_measure_var = yaw_debug_proto.yaw_measure_var();
    yaw_debug_info.vcs_pi_yaw = yaw_debug_proto.vcs_pi_yaw();
    yaw_debug_info.world_pi_yaw = yaw_debug_proto.world_pi_yaw();
    yaw_debug_info.yaw_rate = yaw_debug_proto.yaw_rate();
    yaw_debug_info.yaw_var = yaw_debug_proto.yaw_var();

    return;
}

int DynamicObstacleDebugInfoSerialize(
    const DynamicObstacleDebugInfo &dynamic_obs_debug_info,
    workflow::proto::DynamicObstacleDebugInfo *dynamic_obs_debug_proto)
{
    if (nullptr == dynamic_obs_debug_proto)
    {
        throw InvalidArgumentException(
            "DynamicObstacleDebugInfo dynamic_obs_debug_proto is nullptr !");
        return 0;
    }
    // common part
    dynamic_obs_debug_proto->set_id(dynamic_obs_debug_info.id);
    dynamic_obs_debug_proto->set_frame_id(dynamic_obs_debug_info.frame_id);
    dynamic_obs_debug_proto->set_sensor_id(dynamic_obs_debug_info.sensor_id);

    // position debug info
    const PositionDebugInfo &position_debug_info =
        dynamic_obs_debug_info.pos_debug_info;
    workflow::proto::PositionDebugInfo *p_position_debug_proto =
        dynamic_obs_debug_proto->mutable_pos_debug_info();
    FillPositionDebugInfoProto(p_position_debug_proto, position_debug_info);

    // speed debug info
    const SpeedDebugInfo &speed_debug_info =
        dynamic_obs_debug_info.speed_debug_info;
    workflow::proto::SpeedDebugInfo *p_speed_debug_proto =
        dynamic_obs_debug_proto->mutable_speed_debug_info();
    FillSpeedDebugInfoProto(p_speed_debug_proto, speed_debug_info);

    // size debug info
    const SizeDebugInfo &size_debug_info =
        dynamic_obs_debug_info.size_debug_info;
    workflow::proto::SizeDebugInfo *p_size_debug_proto =
        dynamic_obs_debug_proto->mutable_size_debug_info();
    FillSizeDebugInfoProto(p_size_debug_proto, size_debug_info);

    // yaw debug info
    const YawDebugInfo &yaw_debug_info =
        dynamic_obs_debug_info.yaw_debug_info;
    workflow::proto::YawDebugInfo *p_yaw_debug_proto =
        dynamic_obs_debug_proto->mutable_yaw_debug_info();
    FillYawDebugProto(p_yaw_debug_proto, yaw_debug_info);

    return 1;
}

int DynamicObstacleDebugInfoDeserialize(
    const workflow::proto::DynamicObstacleDebugInfo &dynamic_obs_debug_proto,
    DynamicObstacleDebugInfo &dynamic_obs_debug_info)
{
    // common part
    dynamic_obs_debug_info.id = dynamic_obs_debug_proto.id();
    dynamic_obs_debug_info.frame_id = dynamic_obs_debug_proto.frame_id();
    dynamic_obs_debug_info.sensor_id = dynamic_obs_debug_proto.sensor_id();

    // position debug info
    if (dynamic_obs_debug_proto.has_pos_debug_info())
    {
        PositionDebugInfo &position_debug_info = dynamic_obs_debug_info.pos_debug_info;
        const workflow::proto::PositionDebugInfo &position_debug_proto =
            dynamic_obs_debug_proto.pos_debug_info();
        FillPositionDebugInfo(position_debug_info, position_debug_proto);
    }

    // speed debug info
    if (dynamic_obs_debug_proto.has_speed_debug_info())
    {
        SpeedDebugInfo &speed_debug_info = dynamic_obs_debug_info.speed_debug_info;
        const workflow::proto::SpeedDebugInfo &speed_debug_proto =
            dynamic_obs_debug_proto.speed_debug_info();
        FillSpeedDebugInfo(speed_debug_info, speed_debug_proto);
    }

    // size debug info
    if (dynamic_obs_debug_proto.has_size_debug_info())
    {
        SizeDebugInfo &size_debug_info = dynamic_obs_debug_info.size_debug_info;
        const workflow::proto::SizeDebugInfo &size_debug_proto =
            dynamic_obs_debug_proto.size_debug_info();
        FillSizeDebugInfo(size_debug_info, size_debug_proto);
    }

    //yaw debug info
    if (dynamic_obs_debug_proto.has_yaw_debug_info())
    {
        YawDebugInfo &yaw_debug_info = dynamic_obs_debug_info.yaw_debug_info;
        const workflow::proto::YawDebugInfo &yaw_debug_proto =
            dynamic_obs_debug_proto.yaw_debug_info();
        FillYawDebugInfo(yaw_debug_info, yaw_debug_proto);
    }

    return 1;

}

} // namespace autodrive