//

#include <utils/exception.h>

#include "interface/parking/serializer/hmi_render_serializer.h"
#include "serializer/common_serializer.h"

#include "interface/parking/hmi_render_data.h"
#include "protocol/hmi_render.pb.h"
#include "protocol/fusion_obstacle.pb.h"
#include "protocol/fusion_parking_slots.pb.h"

namespace autodrive {

int HmiRenderSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& hmi_render{data_msg->GetHmiRender()};

  workflow::proto::HmiRender* aptiv_proto =
      proto_msg->mutable_hmi_render();

  auto obs_proto =
      aptiv_proto->mutable_obstacles();
  auto& obs_data{hmi_render.obstacles};
  obs_proto->set_timestamp(obs_data.timestamp);
  for (std::uint32_t ob_idx = 0; ob_idx < obs_data.num_obstacle_list;
       ob_idx++) {
    auto& ob_data{obs_data.obstacle_list.at(ob_idx)};
    workflow::proto::FusionObstacleOutPut* ob_proto =
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
    ob_proto->mutable_uss_pos()->mutable_pos1()->set_x(
        ob_data.uss_pos.pos1.x);
    ob_proto->mutable_uss_pos()->mutable_pos1()->set_y(
        ob_data.uss_pos.pos1.y);
    ob_proto->mutable_uss_pos()->mutable_pos1()->set_z(
        ob_data.uss_pos.pos1.z);
    ob_proto->mutable_uss_pos()->mutable_pos2()->set_x(
        ob_data.uss_pos.pos2.x);
    ob_proto->mutable_uss_pos()->mutable_pos2()->set_y(
        ob_data.uss_pos.pos2.y);
    ob_proto->mutable_uss_pos()->mutable_pos2()->set_z(
        ob_data.uss_pos.pos2.z);
    ob_proto->set_motion_category(
        static_cast<workflow::proto::ObstacleMotionCategory>(
            ob_data.motion_category));
    ob_proto->set_motion_orientation(
        static_cast<workflow::proto::ObstacleMotionOrientation>(
            ob_data.motion_orientation));
  }  // end fusion_obstacles

  auto fusion_park_slots_data{hmi_render.parkingSlots};
  auto parking_proto =
      aptiv_proto->mutable_parking_slots();
  for (std::uint32_t i = 0; i < fusion_park_slots_data.num_parking_slots; ++i) {
    auto& slot_info_data{fusion_park_slots_data.parking_slots.at(i)};
    workflow::proto::parking_slot::ParkingSlotInfo* slot_info_proto =
        parking_proto->add_parking_slots();
    slot_info_proto->set_id(slot_info_data.id);
    slot_info_proto->set_valid(slot_info_data.valid);
    slot_info_proto->set_conf(slot_info_data.conf);

    for (std::uint32_t j = 0; j < slot_info_data.num_polygon_points; ++j) {
      auto& corner_data{slot_info_data.polygon_points.at(j)};
      workflow::proto::parking_slot::CornerInfo* corner_proto =
          slot_info_proto->add_polygon_points();
      corner_proto->mutable_point_vcs()->set_x(corner_data.point_vcs.x);
      corner_proto->mutable_point_vcs()->set_y(corner_data.point_vcs.y);
      corner_proto->mutable_point_vcs()->set_z(corner_data.point_vcs.z);
      corner_proto->mutable_point_world()->set_x(corner_data.point_world.x);
      corner_proto->mutable_point_world()->set_y(corner_data.point_world.y);
      corner_proto->mutable_point_world()->set_z(corner_data.point_world.z);
      corner_proto->mutable_point_cov()->set_x(corner_data.point_cov.x);
      corner_proto->mutable_point_cov()->set_y(corner_data.point_cov.y);
      corner_proto->mutable_point_cov()->set_z(corner_data.point_cov.z);
      corner_proto->set_valid(corner_data.valid);
    }  // end polygon_points
    auto& pose_data{slot_info_data.pose};
    workflow::proto::parking_slot::PoseInfo* pose_proto =
        slot_info_proto->mutable_pose();
    pose_proto->mutable_position_vcs()->set_x(pose_data.position_vcs.x);
    pose_proto->mutable_position_vcs()->set_y(pose_data.position_vcs.y);
    pose_proto->mutable_position_vcs()->set_z(pose_data.position_vcs.z);
    pose_proto->mutable_position_world()->set_x(pose_data.position_world.x);
    pose_proto->mutable_position_world()->set_y(pose_data.position_world.y);
    pose_proto->mutable_position_world()->set_z(pose_data.position_world.z);
    pose_proto->set_length(pose_data.length);
    pose_proto->set_width(pose_data.width);
    pose_proto->set_yaw_vcs(pose_data.yaw_vcs);
    pose_proto->set_yaw_world(pose_data.yaw_world);
    pose_proto->set_slotline_w0(pose_data.slotline_w0);
    pose_proto->set_slotline_w1(pose_data.slotline_w1);
    pose_proto->set_alpha_vcs(pose_data.alpha_vcs);
    pose_proto->set_alpha_world(pose_data.alpha_world);
    for(std::uint32_t j = 0; j < pose_data.num_corners; ++j) {
      workflow::proto::Vector3f* vcs_proto = pose_proto->add_corners_vcs();
      auto& vcs_data{pose_data.corners_vcs.at(j)};
      vcs_proto->set_x(vcs_data.x);
      vcs_proto->set_y(vcs_data.y);
      vcs_proto->set_z(vcs_data.z);
    }
    for(std::uint32_t j = 0; j < pose_data.num_corners; ++j) {
      workflow::proto::Vector3f* wld_proto = pose_proto->add_corners_world();
      auto& wld_data{pose_data.corners_world.at(j)};
      wld_proto->set_x(wld_data.x);
      wld_proto->set_y(wld_data.y);
      wld_proto->set_z(wld_data.z);
    }
    auto& park_rod_data{slot_info_data.parking_rod};
    workflow::proto::parking_slot::ParkingRodInfo* park_rod_proto =
        slot_info_proto->mutable_parking_rod();
    for (size_t j = 0; j < park_rod_data.num_end_points; ++j) {
      auto& vcs_data{park_rod_data.end_points_vcs.at(j)};
      workflow::proto::Vector3f* vcs_proto = park_rod_proto->add_end_points_vcs();
      vcs_proto->set_x(vcs_data.x);
      vcs_proto->set_y(vcs_data.y);
      vcs_proto->set_z(vcs_data.z);
    }
    for (size_t j = 0; j < park_rod_data.num_end_points; ++j) {
      auto& wld_data{park_rod_data.end_points_world.at(j)};
      workflow::proto::Vector3f* wld_proto = park_rod_proto->add_end_points_world();
      wld_proto->set_x(wld_data.x);
      wld_proto->set_y(wld_data.y);
      wld_proto->set_z(wld_data.z);
    }
    park_rod_proto->set_valid(park_rod_data.valid);

    workflow::proto::parking_slot::ParkingLockInfo* lock_info_proto =
        slot_info_proto->mutable_parking_lock_info();
    auto& lock_info_data{slot_info_data.parking_lock_info};
    lock_info_proto->mutable_point_vcs()->set_x(lock_info_data.point_vcs.x);
    lock_info_proto->mutable_point_vcs()->set_y(lock_info_data.point_vcs.y);
    lock_info_proto->mutable_point_vcs()->set_z(lock_info_data.point_vcs.z);

    lock_info_proto->mutable_point_world()->set_x(lock_info_data.point_world.x);
    lock_info_proto->mutable_point_world()->set_y(lock_info_data.point_world.y);
    lock_info_proto->mutable_point_world()->set_z(lock_info_data.point_world.z);

    lock_info_proto->set_valid(lock_info_data.valid);
    lock_info_proto->set_parking_status(
        static_cast<workflow::proto::parking_slot::ParkingLockStatus>(lock_info_data.parking_status));

    auto& boundary_data{slot_info_data.boundary};
    workflow::proto::parking_slot::BoundaryInfo* boundary_proto =
        slot_info_proto->mutable_boundary();
    for (std::uint32_t j = 0; j < boundary_data.num_boundary_points; ++j) {
      workflow::proto::Vector3f* vcs_proto = boundary_proto->add_points_vcs();
      vcs_proto->set_x(boundary_data.points_vcs.at(j).x);
      vcs_proto->set_y(boundary_data.points_vcs.at(j).y);
      vcs_proto->set_z(boundary_data.points_vcs.at(j).z);
    }
    for (std::uint32_t j = 0; j < boundary_data.num_boundary_points; ++j) {
      workflow::proto::Vector3f* wld_proto = boundary_proto->add_points_world();
      wld_proto->set_x(boundary_data.points_world.at(j).x);
      wld_proto->set_y(boundary_data.points_world.at(j).y);
      wld_proto->set_z(boundary_data.points_world.at(j).z);
    }
    for (std::uint32_t j = 0; j < boundary_data.num_boundary_points; ++j) {
      boundary_proto->add_boundaries_status(
        static_cast<workflow::proto::parking_slot::BoundaryStatus>(boundary_data.boundaries_status.at(j)));
    }
    for (std::uint32_t j = 0; j < boundary_data.num_boundary_points; ++j) {
      boundary_proto->add_boundaries_height(boundary_data.boundaries_height.at(j));
    }
    for (std::uint32_t j = 0; j < boundary_data.num_boundary_points; ++j) {
      boundary_proto->add_boundaries_fix_status(
        static_cast<workflow::proto::parking_slot::BoundaryFixStatus>(boundary_data.boundaries_fix_status.at(j)));
    }
    for (std::int32_t j = 0; j < boundary_proto->points_delta_ss_size(); ++j) {
      auto& point_data{boundary_data.points_delta_ss.at(j)};
      workflow::proto::Vector3f* point_proto = boundary_proto->add_points_delta_ss();
      point_proto->set_x(point_data.x);
      point_proto->set_y(point_data.y);
      point_proto->set_z(point_data.z);
    }
    slot_info_proto->set_type(
        static_cast<workflow::proto::parking_slot::ParkingType>(slot_info_data.type));
    slot_info_proto->set_occupancy_status(
        static_cast<workflow::proto::parking_slot::OccupancyStatus>(slot_info_data.occupancy_status));
    slot_info_proto->set_parking_source(
        static_cast<workflow::proto::parking_slot::ParkingSource>(slot_info_data.parking_source));
    slot_info_proto->set_stage(
        static_cast<workflow::proto::parking_slot::ParkingStage>(slot_info_data.stage));
    slot_info_proto->set_occupancy_obs_type(
        static_cast<workflow::proto::parking_slot::OccupyObjType>(slot_info_data.occupancy_obs_type));
    slot_info_proto->set_str_id(slot_info_data.str_id.data());
  }
  for (std::uint32_t i = 0; i < fusion_park_slots_data.num_certificated_slots; ++i) {
    parking_proto->add_certified_slots(fusion_park_slots_data.certified_slots.at(i));
  }

  return 0;
}

int HmiRenderSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto& fusion_obstacle_data{data_msg->MutableHmiRender().obstacles};

  if (proto_msg->hmi_render().has_obstacles()) {
    auto& obst_proto{proto_msg->hmi_render().obstacles()};
    auto& obst_data{fusion_obstacle_data};
    obst_data.timestamp = obst_proto.timestamp();
    obst_data.num_obstacle_list = obst_proto.obstacle_list_size();
    for (std::uint32_t obs_idx = 0; obs_idx < obst_data.num_obstacle_list;
         obs_idx++) {
      auto& output_proto{obst_proto.obstacle_list(obs_idx)};
      auto& output_data{obst_data.obstacle_list.at(obs_idx)};
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
      output_data.veh_sub_type = static_cast<FusionObstacleData::VehicleSubType>(
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

  auto& slot_Info_data{data_msg->MutableHmiRender().parkingSlots};
  slot_Info_data.num_parking_slots = proto_msg->hmi_render().parking_slots().parking_slots_size();
  if (slot_Info_data.num_parking_slots > FusionParkingSlotsData::kMaxParkingSlotsSize) {
    throw OutOfResourceException("parking slots num is out of range");
  }
  for (std::uint32_t i = 0; i < slot_Info_data.num_parking_slots; ++i) {
    auto& slot_data{slot_Info_data.parking_slots.at(i)};
    auto& slot_proto{proto_msg->hmi_render().parking_slots().parking_slots(i)};
    slot_data.id = slot_proto.id();
    slot_data.valid = slot_proto.valid();
    slot_data.conf = slot_proto.conf();
    slot_data.num_polygon_points = slot_proto.polygon_points_size();
    for (std::int32_t j = 0; j < slot_proto.polygon_points_size(); ++j) {
      auto& pp_proto{slot_proto.polygon_points(j)};
      slot_data.polygon_points.at(j).point_cov.x = pp_proto.point_cov().x();
      slot_data.polygon_points.at(j).point_cov.y = pp_proto.point_cov().y();
      slot_data.polygon_points.at(j).point_cov.z = pp_proto.point_cov().z();

      slot_data.polygon_points.at(j).point_world.x = pp_proto.point_world().x();
      slot_data.polygon_points.at(j).point_world.y = pp_proto.point_world().y();
      slot_data.polygon_points.at(j).point_world.z = pp_proto.point_world().z();

      slot_data.polygon_points.at(j).point_vcs.x = pp_proto.point_vcs().x();
      slot_data.polygon_points.at(j).point_vcs.y = pp_proto.point_vcs().y();
      slot_data.polygon_points.at(j).point_vcs.z = pp_proto.point_vcs().z();

      slot_data.valid = slot_proto.valid();
    }  // end polygon_points

    if (slot_proto.has_pose()) {
      auto& pose_data{slot_data.pose};
      auto& pose_proto{slot_proto.pose()};
      pose_data.position_vcs.x = pose_proto.position_vcs().x();
      pose_data.position_vcs.y = pose_proto.position_vcs().y();
      pose_data.position_vcs.z = pose_proto.position_vcs().z();
      pose_data.position_world.x = pose_proto.position_world().x();
      pose_data.position_world.y = pose_proto.position_world().y();
      pose_data.position_world.z = pose_proto.position_world().z();
      pose_data.length = pose_proto.length();
      pose_data.width = pose_proto.width();
      pose_data.slotline_w0 = pose_proto.slotline_w0();
      pose_data.slotline_w1 = pose_proto.slotline_w1();
      pose_data.yaw_vcs = pose_proto.yaw_vcs();
      pose_data.yaw_world = pose_proto.yaw_world();
      pose_data.alpha_vcs = pose_proto.alpha_vcs();
      pose_data.alpha_world = pose_proto.alpha_world();
      pose_data.num_corners = pose_proto.corners_vcs_size();
      for (size_t j = 0; j < pose_data.num_corners; ++j) {
        pose_data.corners_vcs.at(j).x = pose_proto.corners_vcs(j).x();
        pose_data.corners_vcs.at(j).y = pose_proto.corners_vcs(j).y();
        pose_data.corners_vcs.at(j).z = pose_proto.corners_vcs(j).z();
      }

      for (size_t j = 0; j < pose_data.num_corners; ++j) {
        pose_data.corners_world.at(j).x = pose_proto.corners_world(j).x();
        pose_data.corners_world.at(j).y = pose_proto.corners_world(j).y();
        pose_data.corners_world.at(j).z = pose_proto.corners_world(j).z();
      }
    }  // end pose

    if (slot_proto.has_parking_rod()) {
      auto& rod_proto{slot_proto.parking_rod()};
      auto& rod_data{slot_data.parking_rod};
      rod_data.num_end_points = rod_proto.end_points_vcs_size();
      for (std::int32_t j = 0; j < rod_proto.end_points_vcs_size(); ++j) {
        rod_data.end_points_vcs.at(j).x = rod_proto.end_points_vcs(j).x();
        rod_data.end_points_vcs.at(j).y = rod_proto.end_points_vcs(j).y();
        rod_data.end_points_vcs.at(j).z = rod_proto.end_points_vcs(j).z();
      }
      for (std::int32_t j = 0; j < rod_proto.end_points_world_size(); ++j) {
        rod_data.end_points_world.at(j).x = rod_proto.end_points_world(j).x();
        rod_data.end_points_world.at(j).y = rod_proto.end_points_world(j).y();
        rod_data.end_points_world.at(j).z = rod_proto.end_points_world(j).z();
      }
      rod_data.valid = rod_proto.valid();
    }  // end parking_rod

    if (slot_proto.has_parking_lock_info()) {
      auto& lock_info_proto{slot_proto.parking_lock_info()};
      auto& lock_info_date{slot_data.parking_lock_info};

      lock_info_date.point_vcs.x = lock_info_proto.point_vcs().x();
      lock_info_date.point_vcs.y = lock_info_proto.point_vcs().y();
      lock_info_date.point_vcs.z = lock_info_proto.point_vcs().z();

      lock_info_date.point_world.x = lock_info_proto.point_world().x();
      lock_info_date.point_world.y = lock_info_proto.point_world().y();
      lock_info_date.point_world.z = lock_info_proto.point_world().z();

      lock_info_date.parking_status = static_cast<autodrive::ParkingLockStatus>(
          lock_info_proto.parking_status());
      lock_info_date.valid = lock_info_proto.valid();
    }  // end lock info

    if (slot_proto.has_boundary()) {
      auto& boundary_proto{slot_proto.boundary()};
      auto& boundary_data{slot_data.boundary};

      boundary_data.num_boundary_points = boundary_proto.points_vcs_size();
      for (std::int32_t j = 0; j < boundary_proto.points_vcs_size(); ++j) {
        boundary_data.points_vcs.at(j).x = boundary_proto.points_vcs(j).x();
        boundary_data.points_vcs.at(j).y = boundary_proto.points_vcs(j).y();
        boundary_data.points_vcs.at(j).z = boundary_proto.points_vcs(j).z();
      }

      for (std::int32_t j = 0; j < boundary_proto.points_world_size(); ++j) {
        boundary_data.points_world.at(j).x = boundary_proto.points_world(j).x();
        boundary_data.points_world.at(j).y = boundary_proto.points_world(j).y();
        boundary_data.points_world.at(j).z = boundary_proto.points_world(j).z();
      }

      for (std::int32_t j = 0; j < boundary_proto.boundaries_status_size(); ++j) {
        boundary_data.boundaries_status.at(j) =
            static_cast<autodrive::BoundaryStatus>(boundary_proto.boundaries_status(j));
      }

      for (std::int32_t j = 0; j < boundary_proto.boundaries_height_size(); ++j) {
        boundary_data.boundaries_height.at(j) = boundary_proto.boundaries_height(j);
      }

      for (std::int32_t j = 0; j < boundary_proto.boundaries_fix_status_size(); ++j) {
        boundary_data.boundaries_fix_status.at(j) =
            static_cast<autodrive::BoundaryFixStatus>(boundary_proto.boundaries_fix_status(j));
      }

      for (std::int32_t j = 0; j < boundary_proto.points_delta_ss_size(); ++j) {
        auto& point_data{boundary_data.points_delta_ss.at(j)};
        auto& point_proto{boundary_proto.points_delta_ss(j)};
        point_data.x = point_proto.x();
        point_data.y = point_proto.y();
        point_data.z = point_proto.z();
      }
    }  // end boundary

    slot_data.type = static_cast<autodrive::ParkingType>(slot_proto.type());
    slot_data.occupancy_status =
        static_cast<autodrive::OccupancyStatus>(slot_proto.occupancy_status());
    slot_data.parking_source =
        static_cast<autodrive::ParkingSource>(slot_proto.parking_source());
    slot_data.stage = static_cast<autodrive::ParkingStage>(slot_proto.stage());
    slot_data.occupancy_obs_type =
        static_cast<autodrive::OccupyObjType>(slot_proto.occupancy_obs_type());
    auto& value{slot_proto.str_id()};
    memcpy(slot_data.str_id.data(), value.c_str(), value.length());
  }
  slot_Info_data.num_certificated_slots = proto_msg->hmi_render().parking_slots().certified_slots_size();
  if (slot_Info_data.num_certificated_slots > FusionParkingSlotsData::kMaxParkingSlotsSize) {
    throw OutOfResourceException("certificated slots num is out of range");
  }
  for (std::uint32_t i = 0; i < slot_Info_data.num_certificated_slots; ++i) {
    slot_Info_data.certified_slots.at(i) = proto_msg->hmi_render().parking_slots().certified_slots(i);
  }

  return 0;
}
}