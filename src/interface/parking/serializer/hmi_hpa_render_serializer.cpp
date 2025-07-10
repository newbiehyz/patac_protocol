//

#include "interface/parking/serializer/hmi_hpa_render_serializer.h"

#include "interface/parking/hmi_hpa_render_data.h"
#include "protocol/fusion_obstacle.pb.h"
#include "protocol/hmi_hpa_render.pb.h"
#include "protocol/localization.pb.h"
#include "protocol/map_display.pb.h"
#include "protocol/vpa_state.pb.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "type/serializer/mapinfo_header_serialize.h"

namespace autodrive {

int HmiHpaRenderSerializer::Serialize(const message::SerializeMsg *data_ptr,
                                      message::DeserializeMsg *msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto &hmi_hpa_render{data_msg->GetHmiHpaRender()};

  workflow::proto::HmiHpaRender *hmi_hpa_proto =
      proto_msg->mutable_hmi_hpa_render();

  auto obs_proto = hmi_hpa_proto->mutable_obstacles();
  auto &obs_data{hmi_hpa_render.hmi_fusion_obstacles};
  obs_proto->set_timestamp(obs_data.timestamp);
  for (std::uint32_t ob_idx = 0; ob_idx < obs_data.num_obstacle_list;
       ob_idx++) {
    auto &ob_data{obs_data.obstacle_list.at(ob_idx)};
    workflow::proto::FusionObstacleOutPut *ob_proto =
        obs_proto->add_obstacle_list();
    ob_proto->set_age(ob_data.age);
    ob_proto->set_is_updated(ob_data.is_updated);
    ob_proto->set_sensor_source(ob_data.sensor_source);
    ob_proto->set_id(ob_data.id);
    ob_proto->set_vision_id(ob_data.vision_id);
    ob_proto->set_uss_id(ob_data.uss_id);
    ob_proto->set_mono_id(ob_data.mono_id);
    ob_proto->mutable_pos()->set_x(ob_data.pos.x);
    ob_proto->mutable_pos()->set_y(ob_data.pos.y);
    ob_proto->mutable_pos()->set_z(ob_data.pos.z);
    ob_proto->mutable_pos_std()->set_x(ob_data.pos_std.x);
    ob_proto->mutable_pos_std()->set_y(ob_data.pos_std.y);
    ob_proto->mutable_pos_std()->set_z(ob_data.pos_std.z);
    ob_proto->mutable_vel()->set_x(ob_data.vel.x);
    ob_proto->mutable_vel()->set_y(ob_data.vel.y);
    ob_proto->mutable_vel()->set_z(ob_data.vel.z);
    ob_proto->mutable_acc()->set_x(ob_data.acc.x);
    ob_proto->mutable_acc()->set_y(ob_data.acc.y);
    ob_proto->mutable_acc()->set_z(ob_data.acc.z);
    ob_proto->set_heading(ob_data.heading);
    ob_proto->set_angular_speed(ob_data.angular_speed);
    ob_proto->set_obstacle_conf(ob_data.obstacle_conf);
    ob_proto->set_width(ob_data.width);
    ob_proto->set_length(ob_data.length);
    ob_proto->set_height(ob_data.height);
    ob_proto->set_ttc(ob_data.ttc);
    ob_proto->set_type(
        static_cast<workflow::proto::FusionObstacleType>(ob_data.type));
    ob_proto->set_veh_sub_type(
        static_cast<workflow::proto::VehicleSubType>(ob_data.veh_sub_type));
    ob_proto->set_static_sub_type(
        static_cast<workflow::proto::FusionObstacleStaticSubType>(
            ob_data.static_sub_type));
    ob_proto->set_motion_state(
        static_cast<workflow::proto::ObstacleMotionStatus>(
            ob_data.motion_state));
    ob_proto->mutable_uss_pos()->mutable_pos1()->set_x(ob_data.uss_pos.pos1.x);
    ob_proto->mutable_uss_pos()->mutable_pos1()->set_y(ob_data.uss_pos.pos1.y);
    ob_proto->mutable_uss_pos()->mutable_pos1()->set_z(ob_data.uss_pos.pos1.z);
    ob_proto->mutable_uss_pos()->mutable_pos2()->set_x(ob_data.uss_pos.pos2.x);
    ob_proto->mutable_uss_pos()->mutable_pos2()->set_y(ob_data.uss_pos.pos2.y);
    ob_proto->mutable_uss_pos()->mutable_pos2()->set_z(ob_data.uss_pos.pos2.z);
    ob_proto->set_motion_category(
        static_cast<workflow::proto::ObstacleMotionCategory>(
            ob_data.motion_category));
    ob_proto->set_motion_orientation(
        static_cast<workflow::proto::ObstacleMotionOrientation>(
            ob_data.motion_orientation));
  } // end fusion_obstacles

  auto loc_proto = hmi_hpa_proto->mutable_localization();
  auto &loc_data{hmi_hpa_render.hmi_localization};
  {
    workflow::proto::ndm_base::NdmSpec *ndm_proto =
        loc_proto->mutable_ndm_header();
    auto &ndm_data{loc_data.ndm_header};
    ndm_proto->set_version(ndm_data.version);
    ndm_proto->set_date(ndm_data.date);
    ndm_proto->set_timestamp(ndm_data.timestamp);
    ndm_proto->set_seq_id(ndm_data.seq_id);
    ndm_proto->set_projection(ndm_data.projection);
    ndm_proto->set_utm_zone(ndm_data.utm_zone);
    ndm_proto->mutable_base_offset()->set_x(ndm_data.base_offset.x);
    ndm_proto->mutable_base_offset()->set_y(ndm_data.base_offset.y);
    ndm_proto->mutable_base_offset()->set_z(ndm_data.base_offset.z);
  } // end localization ndm_header

  {
    workflow::proto::SpLocation *splical_proto = loc_proto->mutable_location();
    auto &splocal_data{loc_data.location};
    {
      workflow::proto::loc_odo::Odometry *loc_odom_proto =
          splical_proto->mutable_loc_odom();
      auto &loc_odom_data{splocal_data.loc_odom};
      loc_odom_proto->set_stamp(loc_odom_data.stamp);
      loc_odom_proto->mutable_quaternion()->set_x(loc_odom_data.quaternion.x);
      loc_odom_proto->mutable_quaternion()->set_y(loc_odom_data.quaternion.y);
      loc_odom_proto->mutable_quaternion()->set_z(loc_odom_data.quaternion.z);
      loc_odom_proto->mutable_quaternion()->set_w(loc_odom_data.quaternion.w);

      loc_odom_proto->mutable_position()->set_x(loc_odom_data.position.x);
      loc_odom_proto->mutable_position()->set_y(loc_odom_data.position.y);
      loc_odom_proto->mutable_position()->set_z(loc_odom_data.position.z);

      loc_odom_proto->set_seq_id(loc_odom_data.seq_id);

      for (std::uint32_t pc_idx = 0; pc_idx < loc_odom_data.num_pose_cov;
           pc_idx++) {
        loc_odom_proto->add_pose_cov(loc_odom_data.pose_cov.at(pc_idx));
      }

      loc_odom_proto->mutable_linear_velocity()->set_x(
          loc_odom_data.linear_velocity.x);
      loc_odom_proto->mutable_linear_velocity()->set_y(
          loc_odom_data.linear_velocity.y);
      loc_odom_proto->mutable_linear_velocity()->set_z(
          loc_odom_data.linear_velocity.z);

      loc_odom_proto->mutable_angular_velocity()->set_x(
          loc_odom_data.angular_velocity.x);
      loc_odom_proto->mutable_angular_velocity()->set_y(
          loc_odom_data.angular_velocity.y);
      loc_odom_proto->mutable_angular_velocity()->set_z(
          loc_odom_data.angular_velocity.z);

      for (std::uint32_t tc_idx = 0; tc_idx < loc_odom_data.num_twist_cov;
           tc_idx++) {
        loc_odom_proto->add_twist_cov(loc_odom_data.twist_cov.at(tc_idx));
      }

      loc_odom_proto->mutable_map_offset()->set_x(loc_odom_data.map_offset.x);
      loc_odom_proto->mutable_map_offset()->set_y(loc_odom_data.map_offset.y);
      loc_odom_proto->mutable_map_offset()->set_z(loc_odom_data.map_offset.z);

      loc_odom_proto->mutable_velocity()->set_x(loc_odom_data.velocity.x);
      loc_odom_proto->mutable_velocity()->set_y(loc_odom_data.velocity.y);
      loc_odom_proto->mutable_velocity()->set_z(loc_odom_data.velocity.z);

      loc_odom_proto->mutable_acceleration()->set_x(
          loc_odom_data.acceleration.x);
      loc_odom_proto->mutable_acceleration()->set_y(
          loc_odom_data.acceleration.y);
      loc_odom_proto->mutable_acceleration()->set_z(
          loc_odom_data.acceleration.z);
    } // loc_odom}
    splical_proto->set_state(
        static_cast<workflow::proto::SpLocation_SpLocationState>(
            splocal_data.state));
    splical_proto->mutable_velocity()->set_x(splocal_data.velocity.x);
    splical_proto->mutable_velocity()->set_y(splocal_data.velocity.y);
    splical_proto->mutable_velocity()->set_z(splocal_data.velocity.z);

    splical_proto->mutable_angular_vel()->set_x(splocal_data.angular_vel.x);
    splical_proto->mutable_angular_vel()->set_y(splocal_data.angular_vel.y);
    splical_proto->mutable_angular_vel()->set_z(splocal_data.angular_vel.z);

    auto map_display_proto = hmi_hpa_proto->mutable_map_display();
    auto &map_display_data{hmi_hpa_render.hmi_map_display};
    {
      workflow::proto::HmiMapInfo *map_info_proto =
          map_display_proto->mutable_map_info();
      auto &map_info_data{map_display_data.map_info};
      {
        auto &header_data{map_info_data.header};
        workflow::proto::MapCommonHeader *header_proto =
            map_info_proto->mutable_header();
        header_proto->set_map_id(header_data.map_id);
        header_proto->set_map_type(
            static_cast<workflow::proto::MapDataType>(header_data.map_type));
        header_proto->set_utc_time(header_data.utc_time);
        header_proto->set_latitude(header_data.latitude);
        header_proto->set_longitude(header_data.longitude);
        for (std::uint32_t idx = 0; idx < header_data.num_glb_trajectory_point;
             ++idx) {
          auto &glb_trajectory_point_data{
              header_data.glb_trajectory_point.at(idx)};
          workflow::proto::Vector2f *glb_trajectory_point_proto =
              header_proto->add_glb_trajectory_point();
          glb_trajectory_point_proto->set_x(glb_trajectory_point_data.x);
          glb_trajectory_point_proto->set_y(glb_trajectory_point_data.y);
        }
        header_proto->set_map_total_height(header_data.map_total_height);
        header_proto->set_map_total_floor(header_data.map_total_floor);
      }  // end header

      {
        auto &trajectory_data{map_info_data.global_trajectory};
        workflow::proto::GlbTrajectory *trajectory_proto =
            map_info_proto->mutable_global_trajectory();

        for (std::uint32_t gtp_idx = 0;
             gtp_idx < trajectory_data.num_glb_trajectory_point; gtp_idx++) {
          auto &gtp_data{trajectory_data.glb_trajectory_point.at(gtp_idx)};
          workflow::proto::SingleTrajPoint *gtp_proto =
              trajectory_proto->add_glb_trajectory_point();
          gtp_proto->mutable_point()->set_x(gtp_data.point.x);
          gtp_proto->mutable_point()->set_y(gtp_data.point.y);
          gtp_proto->mutable_point()->set_z(gtp_data.point.z);
          gtp_proto->set_floor(gtp_data.floor);
        }  // end for

        trajectory_proto->mutable_start_point()->set_x(
            trajectory_data.start_point.x);
        trajectory_proto->mutable_start_point()->set_y(
            trajectory_data.start_point.y);
        trajectory_proto->mutable_start_point()->set_z(
            trajectory_data.start_point.z);

        trajectory_proto->mutable_end_point()->set_x(
            trajectory_data.end_point.x);
        trajectory_proto->mutable_end_point()->set_y(
            trajectory_data.end_point.y);
        trajectory_proto->mutable_end_point()->set_z(
            trajectory_data.end_point.z);
      }  // end global_trajectory

      {
        auto &gpps_data{map_info_data.global_percep_parking_slots};
        workflow::proto::HmiGlbPrcpPrkgSlots *gpps_proto =
            map_info_proto->mutable_global_percep_parking_slots();
        for (std::uint32_t gps_idx = 0; gps_idx < gpps_data.num_glb_prkg_slots;
             gps_idx++) {
          auto &gps_data{gpps_data.glb_prkg_slots.at(gps_idx)};
          workflow::proto::GlbPrkgSlot *gps_proto =
              gpps_proto->add_glb_prkg_slots();
          gps_proto->set_slot_id(gps_data.slot_id);
          gps_proto->set_slot_status(
              static_cast<workflow::proto::SlotStatus>(gps_data.slot_status));
          gps_proto->set_slot_type(
              static_cast<workflow::proto::SlotType>(gps_data.slot_type));
          gps_proto->set_is_target_slot(gps_data.is_target_slot);

          for (std::uint32_t sp_idx = 0; sp_idx < gps_data.num_slot_points;
               sp_idx++) {
            auto &sp_data{gps_data.slot_points.at(sp_idx)};
            workflow::proto::Vector3f *sp_proto = gps_proto->add_slot_points();
            sp_proto->set_x(sp_data.x);
            sp_proto->set_y(sp_data.y);
            sp_proto->set_z(sp_data.z);
          }  // end slot_points
          gps_proto->set_has_lever(gps_data.has_lever);
          auto &lever_start_point_data{gps_data.lever_start_point};
          workflow::proto::Vector3f *lever_start_point_proto =
              gps_proto->mutable_lever_start_point();
          lever_start_point_proto->set_x(lever_start_point_data.x);
          lever_start_point_proto->set_y(lever_start_point_data.y);
          lever_start_point_proto->set_z(lever_start_point_data.z);
          // end lever start point
          auto &lever_end_point_data{gps_data.lever_end_point};
          workflow::proto::Vector3f *lever_end_point_proto =
              gps_proto->mutable_lever_end_point();
          lever_end_point_proto->set_x(lever_end_point_data.x);
          lever_end_point_proto->set_y(lever_end_point_data.y);
          lever_end_point_proto->set_z(lever_end_point_data.z);
          gps_proto->set_floor(gps_data.floor);
          // end lever end point
        }  // end glb_prkg_slots
      }  // end global_percep_parking_slots
      {
        auto &gpo_data{map_info_data.global_perception_obstacle};
        workflow::proto::HmiGlbPrcpObstacle *gpo_proto =
            map_info_proto->mutable_global_perception_obstacle();
        for (std::uint32_t po_idx = 0; po_idx < gpo_data.num_glb_prcp_obstacle;
             po_idx++) {
          auto &po_data{gpo_data.glb_prcp_obstacle.at(po_idx)};
          workflow::proto::GlbObstacle *po_proto =
              gpo_proto->add_glb_prcp_obstacle();
          for (std::uint32_t border_idx = 0;
               border_idx < po_data.num_border_points; ++border_idx) {
            auto &border_data{po_data.border.at(border_idx)};
            workflow::proto::Vector2f *border_proto = po_proto->add_border();
            border_proto->set_x(border_data.x);
            border_proto->set_y(border_data.y);
          }
          po_proto->mutable_center_position()->set_x(po_data.center_position.x);
          po_proto->mutable_center_position()->set_y(po_data.center_position.y);
          po_proto->mutable_center_position()->set_z(po_data.center_position.z);
          po_proto->set_odom_heading(po_data.odom_heading);
          po_proto->set_obstacle_type(
              static_cast<workflow::proto::Map2HmiObstacleType>(
                  po_data.obstacle_type));
          po_proto->set_danger_level(
              static_cast<workflow::proto::DangerLevel>(po_data.danger_level));
          po_proto->set_target_id(po_data.target_id);
          po_proto->set_obj_status(
              static_cast<workflow::proto::ObjStatus>(po_data.obj_status));
          po_proto->set_floor(po_data.floor);
        }  // end glb_prcp_obstacle
      }  // end global_perception_obstacle

      {
        map_info_proto->set_map_distance(map_info_data.map_distance);
      }  // end map_distance

      {
        map_info_proto->set_rest_map_distance(map_info_data.rest_map_distance);
      }  // end rest_map_distance

      {
        map_info_proto->set_num_speed_bumps(map_info_data.num_speed_bumps);
      }  // end num_speed_bumps
    }  // end map_info

    map_display_proto->set_update_map_info(map_display_data.update_map_info);
    map_display_proto->set_is_match(map_display_data.is_match);
    map_display_proto->set_num_maps(static_cast<int32_t>(map_display_data.num_maps));
    map_display_proto->set_new_id(static_cast<int32_t>(map_display_data.new_id));

    auto &map_header_data{map_display_data.header};
    workflow::proto::MapCommonHeader *map_header_proto =
        map_display_proto->mutable_header();
    autodrive::MapCommonHeaderSerialize(map_header_data, map_header_proto);

    auto vpa_proto = hmi_hpa_proto->mutable_vpa_state();
    auto &vpa_state_data = hmi_hpa_render.hmi_vpa_state;
    {
      vpa_proto->set_avaliable_map_id(vpa_state_data.avaliable_map_id);
      vpa_proto->set_vpa_event(
          static_cast<workflow::proto::VPAEventType>(vpa_state_data.vpa_event));
      vpa_proto->set_hpa_state(
          static_cast<workflow::proto::HPAState>(vpa_state_data.hpa_state));

      vpa_proto->set_mapping_odd_type(
          static_cast<workflow::proto::MappingOddType>(
              vpa_state_data.mapping_odd_type));
      vpa_proto->set_loc_odd_type(static_cast<workflow::proto::LocOddType>(
          vpa_state_data.loc_odd_type));
      vpa_proto->set_map_saving_stage(
          static_cast<workflow::proto::MapSavingStage>(
              vpa_state_data.map_saving_stage));
      vpa_proto->set_local_loc_stage(
          static_cast<workflow::proto::LocalLocStage>(
              vpa_state_data.local_loc_stage));
      vpa_proto->set_error_code(
          static_cast<workflow::proto::ErrorCode>(vpa_state_data.error_code));
      vpa_proto->set_map_deleted_stage(
          static_cast<workflow::proto::MapDeletingStage>(
              vpa_state_data.map_deleted_stage));
      vpa_proto->set_map_stage(
          static_cast<workflow::proto::MappingStage>(vpa_state_data.map_stage));
      vpa_proto->set_map_optimized_stage(
          static_cast<workflow::proto::MapOptimizeStage>(
              vpa_state_data.map_optimized_stage));
      vpa_proto->set_target_parking_slot_id(
          vpa_state_data.target_parking_slot_id);
      vpa_proto->set_map_saving_progress(vpa_state_data.map_saving_progress);
    } // end vpa state data
  }
  return 0;
}

int HmiHpaRenderSerializer::Deserialize(const message::DeserializeMsg *msg_ptr,
                                        message::SerializeMsg *data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto &hmi_hpa_render{data_msg->MutableHmiHpaRender()};
  auto &hmi_hpa_proto = proto_msg->hmi_hpa_render();

  if (proto_msg->hmi_hpa_render().has_obstacles()) {
    auto &obst_proto{proto_msg->hmi_hpa_render().obstacles()};
    auto &obst_data{hmi_hpa_render.hmi_fusion_obstacles};
    obst_data.timestamp = obst_proto.timestamp();
    obst_data.num_obstacle_list = obst_proto.obstacle_list_size();
    for (std::uint32_t obs_idx = 0; obs_idx < obst_data.num_obstacle_list;
         obs_idx++) {
      auto &output_proto{obst_proto.obstacle_list(obs_idx)};
      auto &output_data{obst_data.obstacle_list.at(obs_idx)};
      output_data.age = output_proto.age();
      output_data.is_updated = output_proto.is_updated();
      output_data.sensor_source = output_proto.sensor_source();
      output_data.id = output_proto.id();
      output_data.vision_id = output_proto.vision_id();
      output_data.uss_id = output_proto.uss_id();
      output_data.mono_id = output_proto.mono_id();
      output_data.pos.x = output_proto.pos().x();
      output_data.pos.y = output_proto.pos().y();
      output_data.pos.z = output_proto.pos().z();
      output_data.pos_std.x = output_proto.pos_std().x();
      output_data.pos_std.y = output_proto.pos_std().y();
      output_data.pos_std.z = output_proto.pos_std().z();
      output_data.vel.x = output_proto.vel().x();
      output_data.vel.y = output_proto.vel().y();
      output_data.vel.z = output_proto.vel().z();
      output_data.acc.x = output_proto.acc().x();
      output_data.acc.y = output_proto.acc().y();
      output_data.acc.z = output_proto.acc().z();
      output_data.heading = output_proto.heading();
      output_data.angular_speed = output_proto.angular_speed();
      output_data.obstacle_conf = output_proto.obstacle_conf();
      output_data.width = output_proto.width();
      output_data.length = output_proto.length();
      output_data.height = output_proto.height();
      output_data.ttc = output_proto.ttc();
      output_data.type = static_cast<FusionObstacleData::FusionObstacleType>(
          output_proto.type());
      output_data.veh_sub_type =
          static_cast<FusionObstacleData::VehicleSubType>(
              output_proto.veh_sub_type());
      output_data.static_sub_type =
          static_cast<FusionObstacleData::FusionObstacleStaticSubType>(
              output_proto.static_sub_type());
      output_data.motion_state =
          static_cast<FusionObstacleData::ObstacleMotionStatus>(
              output_proto.motion_state());
      output_data.uss_pos.pos1.x = output_proto.uss_pos().pos1().x();
      output_data.uss_pos.pos1.y = output_proto.uss_pos().pos1().y();
      output_data.uss_pos.pos1.z = output_proto.uss_pos().pos1().z();
      output_data.uss_pos.pos2.x = output_proto.uss_pos().pos2().x();
      output_data.uss_pos.pos2.y = output_proto.uss_pos().pos2().y();
      output_data.uss_pos.pos2.z = output_proto.uss_pos().pos2().z();
      output_data.motion_category =
          static_cast<FusionObstacleData::ObstacleMotionCategory>(
              output_proto.motion_category());
      output_data.motion_orientation =
          static_cast<FusionObstacleData::ObstacleMotionOrientation>(
              output_proto.motion_orientation());
    }
  }

  auto localization_proto = &(hmi_hpa_proto.localization());
  auto &localization_data = hmi_hpa_render.hmi_localization;
  {
    if (localization_proto->has_ndm_header()) {
      auto &ndm_proto{localization_proto->ndm_header()};
      auto &ndm_data{localization_data.ndm_header};
      memcpy(ndm_data.version, ndm_proto.version().c_str(),
             ndm_proto.version().length());
      memcpy(ndm_data.date, ndm_proto.date().c_str(),
             ndm_proto.date().length());
      ndm_data.timestamp = ndm_proto.timestamp();
      ndm_data.seq_id = ndm_proto.seq_id();
      memcpy(ndm_data.projection, ndm_proto.projection().c_str(),
             ndm_proto.projection().length());
      memcpy(ndm_data.utm_zone, ndm_proto.utm_zone().c_str(),
             ndm_proto.utm_zone().length());
      ndm_data.base_offset.x = ndm_proto.base_offset().x();
      ndm_data.base_offset.y = ndm_proto.base_offset().y();
      ndm_data.base_offset.z = ndm_proto.base_offset().z();
    } // end ndm_header

    if (localization_proto->has_location()) {
      auto &splocal_proto{localization_proto->location()};
      auto &splocal_data{localization_data.location};
      if (splocal_proto.has_loc_odom()) {
        auto &odom_proto{splocal_proto.loc_odom()};
        auto &odom_data{splocal_data.loc_odom};
        odom_data.stamp = odom_proto.stamp();
        if (odom_proto.has_quaternion()) {
          odom_data.quaternion.w = odom_proto.quaternion().w();
          odom_data.quaternion.x = odom_proto.quaternion().x();
          odom_data.quaternion.y = odom_proto.quaternion().y();
          odom_data.quaternion.z = odom_proto.quaternion().z();
        }
        if (odom_proto.has_position()) {
          odom_data.position.x = odom_proto.position().x();
          odom_data.position.y = odom_proto.position().y();
          odom_data.position.z = odom_proto.position().z();
        }
        odom_data.seq_id = odom_proto.seq_id();

        odom_data.num_pose_cov = odom_proto.pose_cov_size();
        for (std::uint32_t pc_idx = 0; pc_idx < odom_data.num_pose_cov;
             pc_idx++) {
          odom_data.pose_cov.at(pc_idx) = odom_proto.pose_cov(pc_idx);
        }

        odom_data.linear_velocity.x = odom_proto.linear_velocity().x();
        odom_data.linear_velocity.y = odom_proto.linear_velocity().y();
        odom_data.linear_velocity.z = odom_proto.linear_velocity().z();

        odom_data.angular_velocity.x = odom_proto.angular_velocity().x();
        odom_data.angular_velocity.y = odom_proto.angular_velocity().y();
        odom_data.angular_velocity.z = odom_proto.angular_velocity().z();

        odom_data.num_twist_cov = odom_proto.twist_cov_size();
        for (std::uint32_t tc_idx = 0; tc_idx < odom_data.num_twist_cov;
             tc_idx++) {
          odom_data.twist_cov.at(tc_idx) = odom_proto.twist_cov(tc_idx);
        }

        odom_data.map_offset.x = odom_proto.map_offset().x();
        odom_data.map_offset.y = odom_proto.map_offset().y();
        odom_data.map_offset.z = odom_proto.map_offset().z();

        odom_data.velocity.x = odom_proto.velocity().x();
        odom_data.velocity.y = odom_proto.velocity().y();
        odom_data.velocity.z = odom_proto.velocity().z();

        odom_data.acceleration.x = odom_proto.acceleration().x();
        odom_data.acceleration.y = odom_proto.acceleration().y();
        odom_data.acceleration.z = odom_proto.acceleration().z();
      } // end loc_odom

      splocal_data.state =
          static_cast<LocalizationData::SpLocation::SpLocationState>(
              splocal_proto.state());

      splocal_data.velocity.x = splocal_proto.velocity().x();
      splocal_data.velocity.y = splocal_proto.velocity().y();
      splocal_data.velocity.z = splocal_proto.velocity().z();

      splocal_data.angular_vel.x = splocal_proto.angular_vel().x();
      splocal_data.angular_vel.y = splocal_proto.angular_vel().y();
      splocal_data.angular_vel.z = splocal_proto.angular_vel().z();
    } // end location}

    auto map_display_proto = &(hmi_hpa_proto.map_display());
    auto &map_display_data = hmi_hpa_render.hmi_map_display;
    {
      if (map_display_proto->has_map_info()) {
        auto &map_info_data{map_display_data.map_info};
        auto &map_info_proto{map_display_proto->map_info()};
        if (map_info_proto.has_header()) {
          auto &header_data{map_info_data.header};
          auto &header_proto{map_info_proto.header()};
          header_data.map_id = header_proto.map_id();
          header_data.map_type =
              static_cast<MapDisplayData::MapDataType>(header_proto.map_type());
          header_data.utc_time = header_proto.utc_time();
          header_data.latitude = header_proto.latitude();
          header_data.longitude = header_proto.longitude();
          header_data.num_glb_trajectory_point =
              header_proto.glb_trajectory_point_size();
          for (std::uint32_t idx = 0;
               idx < header_data.num_glb_trajectory_point; ++idx) {
            auto &data{header_data.glb_trajectory_point.at(idx)};
            auto &msg{header_proto.glb_trajectory_point(idx)};
            data.x = msg.x();
            data.y = msg.y();
          }
          header_data.map_total_height = header_proto.map_total_height();
          header_data.map_total_floor = header_proto.map_total_floor();
        }  // end header
        if (map_info_proto.has_global_trajectory()) {
          auto &gt_data{map_info_data.global_trajectory};
          auto &gt_proto{map_info_proto.global_trajectory()};
          gt_data.num_glb_trajectory_point =
              gt_proto.glb_trajectory_point_size();
          for (std::uint32_t gtp_idx = 0;
               gtp_idx < gt_data.num_glb_trajectory_point; gtp_idx++) {
            auto &gt_point_data{gt_data.glb_trajectory_point.at(gtp_idx)};
            auto &gt_point_proto{gt_proto.glb_trajectory_point(gtp_idx)};
            gt_point_data.point.x = gt_point_proto.point().x();
            gt_point_data.point.y = gt_point_proto.point().y();
            gt_point_data.point.z = gt_point_proto.point().z();
            gt_point_data.floor = gt_point_proto.floor();
          }  // end for
          gt_data.start_point.x = gt_proto.start_point().x();
          gt_data.start_point.y = gt_proto.start_point().y();
          gt_data.start_point.z = gt_proto.start_point().z();

          gt_data.end_point.x = gt_proto.end_point().x();
          gt_data.end_point.y = gt_proto.end_point().y();
          gt_data.end_point.z = gt_proto.end_point().z();
        }  // end global_trajectory

        if (map_info_proto.has_global_percep_parking_slots()) {
          auto &park_slots_data{map_info_data.global_percep_parking_slots};
          auto &park_slots_proto{map_info_proto.global_percep_parking_slots()};
          park_slots_data.num_glb_prkg_slots =
              park_slots_proto.glb_prkg_slots_size();
          for (std::uint32_t gps_idx = 0;
               gps_idx < park_slots_data.num_glb_prkg_slots; gps_idx++) {
            auto &slot_data{park_slots_data.glb_prkg_slots.at(gps_idx)};
            auto &slot_proto{park_slots_proto.glb_prkg_slots(gps_idx)};
            slot_data.slot_id = slot_proto.slot_id();
            slot_data.slot_status = static_cast<MapDisplayData::SlotStatus>(
                slot_proto.slot_status());
            slot_data.slot_type =
                static_cast<MapDisplayData::SlotType>(slot_proto.slot_type());
            slot_data.is_target_slot = slot_proto.is_target_slot();

            slot_data.num_slot_points = slot_proto.slot_points_size();
            for (std::uint32_t sp_idx = 0; sp_idx < slot_data.num_slot_points;
                 sp_idx++) {
              auto &slot_point_data{slot_data.slot_points.at(sp_idx)};
              auto &slot_point_proto{slot_proto.slot_points(sp_idx)};
              slot_point_data.x = slot_point_proto.x();
              slot_point_data.y = slot_point_proto.y();
              slot_point_data.z = slot_point_proto.z();
            }  // end slot_points
            slot_data.has_lever = slot_proto.has_lever();
            auto &lever_start_point_data{slot_data.lever_start_point};
            auto &lever_start_point_proto = slot_proto.lever_start_point();
            lever_start_point_data.x = lever_start_point_proto.x();
            lever_start_point_data.y = lever_start_point_proto.y();
            lever_start_point_data.z = lever_start_point_proto.z();
            auto &lever_end_point_data{slot_data.lever_end_point};
            auto &lever_end_point_proto = slot_proto.lever_end_point();
            lever_end_point_data.x = lever_end_point_proto.x();
            lever_end_point_data.y = lever_end_point_proto.y();
            lever_end_point_data.z = lever_end_point_proto.z();
            slot_data.floor = slot_proto.floor();
            // end lever end point
          }  // end glb_prkg_slots
        }  // end global_percep_parking_slots
        if (map_info_proto.has_global_perception_obstacle()) {
          auto &per_obs_data{map_info_data.global_perception_obstacle};
          auto &per_obs_proto{map_info_proto.global_perception_obstacle()};
          per_obs_data.num_glb_prcp_obstacle =
              per_obs_proto.glb_prcp_obstacle_size();
          for (std::uint32_t po_idx = 0;
               po_idx < per_obs_data.num_glb_prcp_obstacle; po_idx++) {
            auto &obs_data{per_obs_data.glb_prcp_obstacle.at(po_idx)};
            auto &obs_proto{per_obs_proto.glb_prcp_obstacle(po_idx)};
            obs_data.num_border_points = obs_proto.border_size();
            for (std::uint32_t border_idx = 0;
                 border_idx < obs_data.num_border_points; ++border_idx) {
              auto &border_data{obs_data.border.at(border_idx)};
              auto &border_msg{obs_proto.border(border_idx)};
              border_data.x = border_msg.x();
              border_data.y = border_msg.y();
            }
            obs_data.center_position.x = obs_proto.center_position().x();
            obs_data.center_position.y = obs_proto.center_position().y();
            obs_data.center_position.z = obs_proto.center_position().z();
            obs_data.odom_heading = obs_proto.odom_heading();
            obs_data.obstacle_type =
                static_cast<MapDisplayData::Map2HmiObstacleType>(
                    obs_proto.obstacle_type());
            obs_data.danger_level = static_cast<MapDisplayData::DangerLevel>(
                obs_proto.danger_level());
            obs_data.target_id = obs_proto.target_id();
            obs_data.obj_status =
                static_cast<MapDisplayData::ObjStatus>(obs_proto.obj_status());
            obs_data.floor = obs_proto.floor();
          }  // end glb_prcp_obstacle
        }  // end global_perception_obstacle

        {
          map_info_data.map_distance = map_info_proto.map_distance();
        }  // end map_distance

        {
          map_info_data.rest_map_distance = map_info_proto.rest_map_distance();
        }  // end rest_map_distance

        {
          map_info_data.num_speed_bumps = map_info_proto.num_speed_bumps();
        }  // end num_
      }  // end map_info

      map_display_data.update_map_info = map_display_proto->update_map_info();
      map_display_data.is_match = map_display_proto->is_match();
      map_display_data.new_id = static_cast<std::int32_t>(map_display_proto->new_id());

      auto &map_header_data{map_display_data.header};
      auto &map_header_proto{map_display_proto->header()};
      autodrive::MapCommonHeaderDeserialize(map_header_proto,
                                            map_header_data);
                                            
    }

    auto vpa_state_proto = &(hmi_hpa_proto.vpa_state());
    auto &vpa_state_data = hmi_hpa_render.hmi_vpa_state;
    {
      vpa_state_data.avaliable_map_id = vpa_state_proto->avaliable_map_id();
      vpa_state_data.vpa_event =
          static_cast<VpaStateData::VPAEventType>(vpa_state_proto->vpa_event());
      vpa_state_data.hpa_state =
          static_cast<VpaStateData::HPAState>(vpa_state_proto->hpa_state());
      vpa_state_data.mapping_odd_type =
          static_cast<VpaStateData::MappingOddType>(
              vpa_state_proto->mapping_odd_type());
      vpa_state_data.loc_odd_type = static_cast<VpaStateData::LocOddType>(
          vpa_state_proto->loc_odd_type());
      vpa_state_data.map_saving_stage =
          static_cast<VpaStateData::MapSavingStage>(
              vpa_state_proto->map_saving_stage());
      vpa_state_data.local_loc_stage = static_cast<VpaStateData::LocalLocStage>(
          vpa_state_proto->local_loc_stage());
      vpa_state_data.error_code =
          static_cast<VpaStateData::ErrorCode>(vpa_state_proto->error_code());
      vpa_state_data.map_deleted_stage =
          static_cast<VpaStateData::MapDeletingStage>(
              vpa_state_proto->map_deleted_stage());
      vpa_state_data.map_stage =
          static_cast<VpaStateData::MappingStage>(vpa_state_proto->map_stage());
      vpa_state_data.map_optimized_stage =
          static_cast<VpaStateData::MapOptimizeStage>(
              vpa_state_proto->map_optimized_stage());
      vpa_state_data.target_parking_slot_id =
          vpa_state_proto->target_parking_slot_id();
      vpa_state_data.map_saving_progress =
          vpa_state_proto->map_saving_progress();
    }
  }
  return 0;
}
} // namespace autodrive
