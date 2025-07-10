//
#include "interface/parking/serializer/reference_lines_serializer.h"

#include <utils/exception.h>

#include "interface/parking/map_objects_data.h"
#include "interface/parking/reference_lines_data.h"
#include "protocol/map_objects.pb.h"
#include "protocol/refrence_lines.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int ReferenceLinesSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& ref_line_data{data_msg->GetReferenceLines()};

  {
    auto& waypoint_raw_data{ref_line_data.waypoint_frame_global_raw};
    workflow::proto::RefLineWayPointFrameGlobal* waypoint_raw_proto =
        proto_msg->mutable_waypoint_frame_global_raw();
    waypoint_raw_proto->set_waypoint_id(waypoint_raw_data.waypoint_id);
    for (std::uint32_t wp_idx = 0; wp_idx < waypoint_raw_data.num_waypoints;
         wp_idx++) {
      auto& wp_data{waypoint_raw_data.way_points.at(wp_idx)};
      workflow::proto::RefLineWayPointGlobal* wp_proto =
          waypoint_raw_proto->add_way_points();
      wp_proto->mutable_waypoint_position()->set_x(wp_data.waypoint_position.x);
      wp_proto->mutable_waypoint_position()->set_y(wp_data.waypoint_position.y);
      wp_proto->mutable_waypoint_position()->set_z(wp_data.waypoint_position.z);
      wp_proto->set_waypoint_yaw(wp_data.waypoint_yaw);
      wp_proto->set_waypoint_pitch(wp_data.waypoint_pitch);
      wp_proto->set_waypoint_roll(wp_data.waypoint_roll);
      wp_proto->set_waypoint_curv(wp_data.waypoint_curv);
      wp_proto->set_waypoint_curvrate(wp_data.waypoint_curvrate);
      wp_proto->set_waypoint_speed(wp_data.waypoint_speed);
      wp_proto->set_waypoint_num(wp_data.waypoint_num);
      wp_proto->set_waypoint_dist(wp_data.waypoint_dist);
      wp_proto->set_waypoint_dir(
          static_cast<workflow::proto::RefLineWayPointGlobal_WaypointDir>(
              wp_data.waypoint_dir));
      wp_proto->set_waypoint_locked(wp_data.waypoint_locked);
      wp_proto->set_is_map_origin(wp_data.is_map_origin);
      wp_proto->set_is_intersection(wp_data.is_intersection);
      wp_proto->set_left_turning_light_status(
          static_cast<workflow::proto::TurningLightStatus>(
              wp_data.left_turning_light_status));
      wp_proto->set_right_turning_light_status(
          static_cast<workflow::proto::TurningLightStatus>(
              wp_data.right_turning_light_status));
      wp_proto->set_waypoint_floor(wp_data.waypoint_floor);
      wp_proto->set_is_parking_gate(wp_data.is_parking_gate);
      wp_proto->set_is_speed_bump(wp_data.is_speed_bump);
      wp_proto->set_left_bound_distance(wp_data.left_bound_distance);
      wp_proto->set_right_bound_distance(wp_data.right_bound_distance);
    }  // end way_points
  }    // end waypoint_frame_global_raw

  {
    workflow::proto::RefLineWayPointFrameGlobal* waypoint_proto =
        proto_msg->mutable_waypoint_frame_global();
    auto& waypoint_data{ref_line_data.waypoint_frame_global};
    waypoint_proto->set_waypoint_id(waypoint_data.waypoint_id);
    for (std::uint32_t wp_idx = 0; wp_idx < waypoint_data.num_waypoints;
         wp_idx++) {
      auto& wp_data{waypoint_data.way_points.at(wp_idx)};
      workflow::proto::RefLineWayPointGlobal* wp_proto =
          waypoint_proto->add_way_points();
      wp_proto->mutable_waypoint_position()->set_x(wp_data.waypoint_position.x);
      wp_proto->mutable_waypoint_position()->set_y(wp_data.waypoint_position.y);
      wp_proto->mutable_waypoint_position()->set_z(wp_data.waypoint_position.z);
      wp_proto->set_waypoint_yaw(wp_data.waypoint_yaw);
      wp_proto->set_waypoint_pitch(wp_data.waypoint_pitch);
      wp_proto->set_waypoint_roll(wp_data.waypoint_roll);
      wp_proto->set_waypoint_curv(wp_data.waypoint_curv);
      wp_proto->set_waypoint_curvrate(wp_data.waypoint_curvrate);
      wp_proto->set_waypoint_speed(wp_data.waypoint_speed);
      wp_proto->set_waypoint_num(wp_data.waypoint_num);
      wp_proto->set_waypoint_dist(wp_data.waypoint_dist);
      wp_proto->set_waypoint_dir(
          static_cast<workflow::proto::RefLineWayPointGlobal_WaypointDir>(
              wp_data.waypoint_dir));
      wp_proto->set_waypoint_locked(wp_data.waypoint_locked);
      wp_proto->set_is_map_origin(wp_data.is_map_origin);
      wp_proto->set_waypoint_floor(wp_data.waypoint_floor);
      wp_proto->set_is_parking_gate(wp_data.is_parking_gate);
      wp_proto->set_is_speed_bump(wp_data.is_speed_bump);
      wp_proto->set_left_bound_distance(wp_data.left_bound_distance);
      wp_proto->set_right_bound_distance(wp_data.right_bound_distance);
    }  // end way_points
  }    // end waypoint_frame_global

  {
    auto& slot_data = ref_line_data.parking_slot;
    workflow::proto::GlbPrkgSlot* slot_msg = proto_msg->mutable_parking_slot();
    slot_msg->set_slot_id(slot_data.slot_id);
    slot_msg->set_slot_status(
        static_cast<workflow::proto::SlotStatus>(slot_data.slot_status));
    slot_msg->set_slot_type(
        static_cast<workflow::proto::SlotType>(slot_data.slot_type));
    slot_msg->set_is_target_slot(slot_data.is_target_slot);

    for (std::uint32_t sp_idx = 0; sp_idx < slot_data.num_slot_points;
         sp_idx++) {
      auto& sp_data{slot_data.slot_points.at(sp_idx)};
      workflow::proto::Vector3f* sp_proto = slot_msg->add_slot_points();
      sp_proto->set_x(sp_data.x);
      sp_proto->set_y(sp_data.y);
      sp_proto->set_z(sp_data.z);
    }  // end slot_points
  }    // end parking_slot

  {
    auto& line_data{ref_line_data.reference_line};
    workflow::proto::RefLine* line_proto = proto_msg->mutable_reference_line();
    line_proto->set_map_floor_num(line_data.map_floor_num);
    line_proto->set_header(line_data.header.data(), line_data.num_header);

    for (std::uint8_t rp_idx = 0; rp_idx < line_data.num_reference_points;
         rp_idx++) {
      auto& point_data{line_data.reference_points.at(rp_idx)};
      workflow::proto::ReferenceLinePoint* point_proto =
          line_proto->add_reference_points();
      {
        auto& odo_data{point_data.odo};
        workflow::proto::loc_odo::Odometry* odo_proto =
            point_proto->mutable_odo();
        odo_proto->set_stamp(odo_data.stamp);

        {
          auto& quat_data{odo_data.quaternion};
          workflow::proto::loc_odo::Quaternion4d* quat_proto =
              odo_proto->mutable_quaternion();
          quat_proto->set_x(quat_data.x);
          quat_proto->set_y(quat_data.y);
          quat_proto->set_z(quat_data.z);
          quat_proto->set_w(quat_data.w);
        }  // end quaternion

        odo_proto->mutable_position()->set_x(odo_data.position.x);
        odo_proto->mutable_position()->set_y(odo_data.position.y);
        odo_proto->mutable_position()->set_z(odo_data.position.z);

        odo_proto->set_seq_id(odo_data.seq_id);

        for (std::uint32_t pc_idx = 0; pc_idx < odo_data.num_pose_cov;
             pc_idx++) {
          odo_proto->add_pose_cov(odo_data.pose_cov.at(pc_idx));
        }

        odo_proto->mutable_linear_velocity()->set_x(odo_data.linear_velocity.x);
        odo_proto->mutable_linear_velocity()->set_y(odo_data.linear_velocity.y);
        odo_proto->mutable_linear_velocity()->set_z(odo_data.linear_velocity.z);

        odo_proto->mutable_angular_velocity()->set_x(
            odo_data.angular_velocity.x);
        odo_proto->mutable_angular_velocity()->set_y(
            odo_data.angular_velocity.y);
        odo_proto->mutable_angular_velocity()->set_z(
            odo_data.angular_velocity.z);

        for (std::uint32_t tc_idx = 0; tc_idx < odo_data.num_twist_cov;
             tc_idx++) {
          odo_proto->add_twist_cov(odo_data.twist_cov.at(tc_idx));
        }

        odo_proto->mutable_map_offset()->set_x(odo_data.map_offset.x);
        odo_proto->mutable_map_offset()->set_y(odo_data.map_offset.y);
        odo_proto->mutable_map_offset()->set_z(odo_data.map_offset.z);

        odo_proto->mutable_velocity()->set_x(odo_data.velocity.x);
        odo_proto->mutable_velocity()->set_y(odo_data.velocity.y);
        odo_proto->mutable_velocity()->set_z(odo_data.velocity.z);

        odo_proto->mutable_acceleration()->set_x(odo_data.acceleration.x);
        odo_proto->mutable_acceleration()->set_y(odo_data.acceleration.y);
        odo_proto->mutable_acceleration()->set_z(odo_data.acceleration.z);
        odo_proto->set_floor(odo_data.floor);
      }  // end odo

      point_proto->set_pitch(point_data.pitch);
      point_proto->set_num_sates(point_data.num_sates);
      point_proto->set_floor_num(point_data.floor_num);
      point_proto->set_vertex_type(
          static_cast<workflow::proto::VertexType>(point_data.vertex_type));
      point_proto->set_has_map(point_data.has_map);
      point_proto->set_init_loc_region(point_data.init_loc_region);
      point_proto->set_str_obj_id(point_data.str_obj_id.data(),
                                  point_data.num_str_obj_id);
    }  // end reference_points
  }    // end reference_line

  {
    auto& ndm_data{ref_line_data.ndm_header};
    workflow::proto::ndm_base::NdmSpec* ndm_proto =
        proto_msg->mutable_ndm_header();
    ndm_proto->set_version(ndm_data.version);
    ndm_proto->set_date(ndm_data.date);
    ndm_proto->set_timestamp(ndm_data.timestamp);
    ndm_proto->set_seq_id(ndm_data.seq_id);
    ndm_proto->set_projection(ndm_data.projection);
    ndm_proto->set_utm_zone(ndm_data.utm_zone);
    ndm_proto->mutable_base_offset()->set_x(ndm_data.base_offset.x);
    ndm_proto->mutable_base_offset()->set_y(ndm_data.base_offset.y);
    ndm_proto->mutable_base_offset()->set_z(ndm_data.base_offset.z);
  }  // end ndm_header

  return 0;
}

int ReferenceLinesSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& ref_line_data{data_msg->MutableReferenceLines()};

  if (proto_msg->has_waypoint_frame_global_raw()) {
    auto& waypoint_raw_data{ref_line_data.waypoint_frame_global_raw};
    auto& waypoint_raw_proto{proto_msg->waypoint_frame_global_raw()};
    waypoint_raw_data.waypoint_id = waypoint_raw_proto.waypoint_id();
    waypoint_raw_data.num_waypoints = waypoint_raw_proto.way_points_size();
    for (std::uint32_t wp_idx = 0; wp_idx < waypoint_raw_data.num_waypoints;
         wp_idx++) {
      auto& way_point_data{waypoint_raw_data.way_points.at(wp_idx)};
      auto& way_point_proto{waypoint_raw_proto.way_points(wp_idx)};
      way_point_data.waypoint_position.x =
          way_point_proto.waypoint_position().x();
      way_point_data.waypoint_position.y =
          way_point_proto.waypoint_position().y();
      way_point_data.waypoint_position.z =
          way_point_proto.waypoint_position().z();
      way_point_data.waypoint_yaw = way_point_proto.waypoint_yaw();
      way_point_data.waypoint_pitch = way_point_proto.waypoint_pitch();
      way_point_data.waypoint_roll = way_point_proto.waypoint_roll();
      way_point_data.waypoint_curv = way_point_proto.waypoint_curv();
      way_point_data.waypoint_curvrate = way_point_proto.waypoint_curvrate();
      way_point_data.waypoint_speed = way_point_proto.waypoint_speed();
      way_point_data.waypoint_num = way_point_proto.waypoint_num();
      way_point_data.waypoint_dist = way_point_proto.waypoint_dist();
      way_point_data.waypoint_dir =
          static_cast<ReferenceLinesData::RefLineWayPointGlobal::WaypointDir>(
              way_point_proto.waypoint_dir());
      way_point_data.waypoint_locked = way_point_proto.waypoint_locked();
      way_point_data.is_map_origin = way_point_proto.is_map_origin();
      way_point_data.is_intersection = way_point_proto.is_intersection();
      way_point_data.left_turning_light_status =
          static_cast<OdometryData::TurningLightStatus>(
              way_point_proto.left_turning_light_status());
      way_point_data.right_turning_light_status =
          static_cast<OdometryData::TurningLightStatus>(
              way_point_proto.right_turning_light_status());
      way_point_data.waypoint_floor=way_point_proto.waypoint_floor();
      way_point_data.is_parking_gate=way_point_proto.is_parking_gate();
      way_point_data.is_speed_bump=way_point_proto.is_speed_bump();
      way_point_data.left_bound_distance=way_point_proto.left_bound_distance();
      way_point_data.right_bound_distance=way_point_proto.right_bound_distance();
    }  // end way_points - for
  }    // end waypoint_frame_global_raw

  if (proto_msg->has_waypoint_frame_global()) {
    auto& waypoint_data{ref_line_data.waypoint_frame_global};
    auto& waypoint_proto{proto_msg->waypoint_frame_global()};
    waypoint_data.waypoint_id = waypoint_proto.waypoint_id();
    waypoint_data.num_waypoints = waypoint_proto.way_points_size();
    for (std::uint32_t wp_idx = 0; wp_idx < waypoint_data.num_waypoints;
         wp_idx++) {
      auto& way_point_data{waypoint_data.way_points.at(wp_idx)};
      auto& way_point_proto{waypoint_proto.way_points(wp_idx)};
      way_point_data.waypoint_position.x =
          way_point_proto.waypoint_position().x();
      way_point_data.waypoint_position.y =
          way_point_proto.waypoint_position().y();
      way_point_data.waypoint_position.z =
          way_point_proto.waypoint_position().z();
      way_point_data.waypoint_yaw = way_point_proto.waypoint_yaw();
      way_point_data.waypoint_pitch = way_point_proto.waypoint_pitch();
      way_point_data.waypoint_roll = way_point_proto.waypoint_roll();
      way_point_data.waypoint_curv = way_point_proto.waypoint_curv();
      way_point_data.waypoint_curvrate = way_point_proto.waypoint_curvrate();
      way_point_data.waypoint_speed = way_point_proto.waypoint_speed();
      way_point_data.waypoint_num = way_point_proto.waypoint_num();
      way_point_data.waypoint_dist = way_point_proto.waypoint_dist();
      way_point_data.waypoint_dir =
          static_cast<ReferenceLinesData::RefLineWayPointGlobal::WaypointDir>(
              way_point_proto.waypoint_dir());
      way_point_data.waypoint_locked = way_point_proto.waypoint_locked();
      way_point_data.is_map_origin = way_point_proto.is_map_origin();
      way_point_data.waypoint_floor=way_point_proto.waypoint_floor();
      way_point_data.is_parking_gate=way_point_proto.is_parking_gate();
      way_point_data.is_speed_bump=way_point_proto.is_speed_bump();
      way_point_data.left_bound_distance=way_point_proto.left_bound_distance();
      way_point_data.right_bound_distance=way_point_proto.right_bound_distance();
    }  // end way_points - for
  }    // end waypoint_frame_global

  if (proto_msg->has_parking_slot()) {
    auto& slot_data{ref_line_data.parking_slot};
    auto& slot_proto{proto_msg->parking_slot()};
    slot_data.slot_id = slot_proto.slot_id();
    slot_data.slot_status =
        static_cast<MapDisplayData::SlotStatus>(slot_proto.slot_status());
    slot_data.slot_type =
        static_cast<MapDisplayData::SlotType>(slot_proto.slot_type());
    slot_data.is_target_slot = slot_proto.is_target_slot();

    slot_data.num_slot_points = slot_proto.slot_points_size();
    for (std::uint32_t sp_idx = 0; sp_idx < slot_data.num_slot_points;
         sp_idx++) {
      auto& slot_point_data{slot_data.slot_points.at(sp_idx)};
      auto& slot_point_proto{slot_proto.slot_points(sp_idx)};
      slot_point_data.x = slot_point_proto.x();
      slot_point_data.y = slot_point_proto.y();
      slot_point_data.z = slot_point_proto.z();
    }  // end slot_points
  }    // end parking_slot

  if (proto_msg->has_reference_line()) {
    auto& line_data{ref_line_data.reference_line};
    auto& line_proto{proto_msg->reference_line()};
    line_data.map_floor_num = line_proto.map_floor_num();
    std::string str_header{line_proto.header()};
    line_data.num_header = str_header.length();
    memcpy(line_data.header.data(), str_header.c_str(), line_data.num_header);
    line_data.num_reference_points = line_proto.reference_points_size();
    for (std::uint8_t rp_idx = 0; rp_idx < line_data.num_reference_points;
         rp_idx++) {
      auto& ref_point_data{line_data.reference_points.at(rp_idx)};
      auto& ref_point_proto{line_proto.reference_points(rp_idx)};

      if (ref_point_proto.has_odo()) {
        auto& odo_data{ref_point_data.odo};
        auto& odo_proto{ref_point_proto.odo()};
        odo_data.stamp = odo_proto.stamp();

        if (odo_proto.has_quaternion()) {
          odo_data.quaternion.x = odo_proto.quaternion().x();
          odo_data.quaternion.y = odo_proto.quaternion().y();
          odo_data.quaternion.z = odo_proto.quaternion().z();
          odo_data.quaternion.w = odo_proto.quaternion().w();
        }  // end quaternion

        if (odo_proto.has_position()) {
          odo_data.position.x = odo_proto.position().x();
          odo_data.position.y = odo_proto.position().y();
          odo_data.position.z = odo_proto.position().z();
        }

        odo_data.seq_id = odo_proto.seq_id();
        odo_data.num_pose_cov = odo_proto.pose_cov_size();
        for (std::uint32_t pc_idx = 0; pc_idx < odo_data.num_pose_cov;
             pc_idx++) {
          odo_data.pose_cov.at(pc_idx) = odo_proto.pose_cov(pc_idx);
        }  // end pose_cov

        if (odo_proto.has_linear_velocity()) {
          odo_data.linear_velocity.x = odo_proto.linear_velocity().x();
          odo_data.linear_velocity.y = odo_proto.linear_velocity().y();
          odo_data.linear_velocity.z = odo_proto.linear_velocity().z();
        }  // end linear_velocity

        if (odo_proto.has_angular_velocity()) {
          odo_data.angular_velocity.x = odo_proto.angular_velocity().x();
          odo_data.angular_velocity.y = odo_proto.angular_velocity().y();
          odo_data.angular_velocity.z = odo_proto.angular_velocity().z();
        }  // end linear_velocity

        odo_data.num_twist_cov = odo_proto.twist_cov_size();
        for (std::uint32_t tc_idx = 0; tc_idx < odo_data.num_twist_cov;
             tc_idx++) {
          odo_data.twist_cov.at(tc_idx) = odo_proto.twist_cov(tc_idx);
        }  // end pose_cov

        if (odo_proto.has_map_offset()) {
          odo_data.map_offset.x = odo_proto.map_offset().x();
          odo_data.map_offset.y = odo_proto.map_offset().y();
          odo_data.map_offset.z = odo_proto.map_offset().z();
        }  // end map_offset

        if (odo_proto.has_velocity()) {
          odo_data.velocity.x = odo_proto.velocity().x();
          odo_data.velocity.y = odo_proto.velocity().y();
          odo_data.velocity.z = odo_proto.velocity().z();
        }  // end map_offset

        if (odo_proto.has_acceleration()) {
          odo_data.acceleration.x = odo_proto.acceleration().x();
          odo_data.acceleration.y = odo_proto.acceleration().y();
          odo_data.acceleration.z = odo_proto.acceleration().z();
        }  // end map_offset
        odo_data.floor = odo_proto.floor();
      }    // end odo

      ref_point_data.pitch = ref_point_proto.pitch();
      ref_point_data.num_sates = ref_point_proto.num_sates();
      ref_point_data.floor_num = ref_point_proto.floor_num();
      ref_point_data.vertex_type = static_cast<
          autodrive::ReferenceLinesData::ReferenceLinePoint::VertexType>(
          ref_point_proto.vertex_type());
      ref_point_data.has_map = ref_point_proto.has_map();
      ref_point_data.init_loc_region = ref_point_proto.init_loc_region();

      std::string str_id{ref_point_proto.str_obj_id()};
      ref_point_data.num_str_obj_id = str_id.length();
      memcpy(ref_point_data.str_obj_id.data(), str_id.c_str(),
             ref_point_data.num_str_obj_id);
    }  // end reference_points
  }    // end reference_line

  if (proto_msg->has_ndm_header()) {
    auto& ndm_data{ref_line_data.ndm_header};
    auto& ndm_proto{proto_msg->ndm_header()};
    std::string str_version{ndm_proto.version()};
    memcpy(ndm_data.version, str_version.c_str(), str_version.length());
    std::string str_data{ndm_proto.date()};
    memcpy(ndm_data.date, str_data.c_str(), str_data.length());
    ndm_data.timestamp = ndm_proto.timestamp();
    ndm_data.seq_id = ndm_proto.seq_id();
    std::string str_projection{ndm_proto.projection()};
    memcpy(ndm_data.projection, str_projection.c_str(),
           str_projection.length());
    std::string str_utm_zone{ndm_proto.utm_zone()};
    memcpy(ndm_data.utm_zone, str_utm_zone.c_str(), str_utm_zone.length());
    ndm_data.base_offset.x = ndm_proto.base_offset().x();
    ndm_data.base_offset.y = ndm_proto.base_offset().y();
    ndm_data.base_offset.z = ndm_proto.base_offset().z();
  }  // end ndm_header

  return 0;
}

}  // namespace autodrive