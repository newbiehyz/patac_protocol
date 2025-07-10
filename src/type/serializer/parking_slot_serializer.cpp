//

#include "type/serializer/parking_slot_serializer.h"

namespace autodrive {

int ParkingSlotSerialize(
    const ParkingSlotInfo& slot_data,
    workflow::proto::parking_slot::ParkingSlotInfo* slot_proto) {
  slot_proto->set_id(slot_data.id);
  slot_proto->set_valid(slot_data.valid);
  slot_proto->set_conf(slot_data.conf);

  for (std::uint32_t pp_idx = 0; pp_idx < slot_data.num_polygon_points;
       pp_idx++) {
    auto& polygon_data{slot_data.polygon_points.at(pp_idx)};
    workflow::proto::parking_slot::CornerInfo* polygon_proto =
        slot_proto->add_polygon_points();
    polygon_proto->mutable_point_vcs()->set_x(polygon_data.point_vcs.x);
    polygon_proto->mutable_point_vcs()->set_y(polygon_data.point_vcs.y);
    polygon_proto->mutable_point_vcs()->set_z(polygon_data.point_vcs.z);
    polygon_proto->mutable_point_world()->set_x(polygon_data.point_world.x);
    polygon_proto->mutable_point_world()->set_y(polygon_data.point_world.y);
    polygon_proto->mutable_point_world()->set_z(polygon_data.point_world.z);
    polygon_proto->mutable_point_cov()->set_x(polygon_data.point_cov.x);
    polygon_proto->mutable_point_cov()->set_y(polygon_data.point_cov.y);
    polygon_proto->mutable_point_cov()->set_z(polygon_data.point_cov.z);
    polygon_proto->set_valid(polygon_data.valid);
  }  // end polygon_points

  {
    auto& pose_data{slot_data.pose};
    workflow::proto::parking_slot::PoseInfo* pose_proto =
        slot_proto->mutable_pose();
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

    for (std::uint32_t c_idx = 0; c_idx < pose_data.num_corners; c_idx++) {
      auto& vcs_data{pose_data.corners_vcs.at(c_idx)};
      workflow::proto::Vector3f* vcs_proto = pose_proto->add_corners_vcs();
      vcs_proto->set_x(vcs_data.x);
      vcs_proto->set_y(vcs_data.y);
      vcs_proto->set_z(vcs_data.z);

      auto& world_data{pose_data.corners_world.at(c_idx)};
      workflow::proto::Vector3f* world_proto = pose_proto->add_corners_world();
      world_proto->set_x(world_data.x);
      world_proto->set_y(world_data.y);
      world_proto->set_z(world_data.z);
    }  // end corners_vcs & corners_world
  }    // end pose

  {
    auto& rod_data{slot_data.parking_rod};
    workflow::proto::parking_slot::ParkingRodInfo* rod_proto =
        slot_proto->mutable_parking_rod();
    rod_proto->set_valid(rod_data.valid);
    for (std::uint32_t pr_idx = 0; pr_idx < rod_data.num_end_points; pr_idx++) {
      auto& vcs_point_data{rod_data.end_points_vcs.at(pr_idx)};
      workflow::proto::Vector3f* vcs_point_proto =
          rod_proto->add_end_points_vcs();
      vcs_point_proto->set_x(vcs_point_data.x);
      vcs_point_proto->set_y(vcs_point_data.y);
      vcs_point_proto->set_z(vcs_point_data.z);

      auto& world_point_data{rod_data.end_points_world.at(pr_idx)};
      workflow::proto::Vector3f* world_point_proto =
          rod_proto->add_end_points_world();
      world_point_proto->set_x(world_point_data.x);
      world_point_proto->set_y(world_point_data.y);
      world_point_proto->set_z(world_point_data.z);
    }  //  end end_points_vcs & end_points_world
  }    // end parking_rod

  {
    auto& lock_data{slot_data.parking_lock_info};
    workflow::proto::parking_slot::ParkingLockInfo* lock_proto =
        slot_proto->mutable_parking_lock_info();
    lock_proto->mutable_point_vcs()->set_x(lock_data.point_vcs.x);
    lock_proto->mutable_point_vcs()->set_y(lock_data.point_vcs.y);
    lock_proto->mutable_point_vcs()->set_z(lock_data.point_vcs.z);

    lock_proto->mutable_point_world()->set_x(lock_data.point_world.x);
    lock_proto->mutable_point_world()->set_y(lock_data.point_world.y);
    lock_proto->mutable_point_world()->set_z(lock_data.point_world.z);

    lock_proto->set_parking_status(
        static_cast<workflow::proto::parking_slot::ParkingLockStatus>(
            lock_data.parking_status));
    lock_proto->set_valid(lock_data.valid);
  }  // end parking_lock_info

  {
    auto& boundary_data{slot_data.boundary};
    workflow::proto::parking_slot::BoundaryInfo* boundary_proto =
        slot_proto->mutable_boundary();
    for (std::uint32_t b_idx = 0; b_idx < boundary_data.num_boundary_points;
         b_idx++) {
      auto& vcs_data{boundary_data.points_vcs.at(b_idx)};
      workflow::proto::Vector3f* vcs_proto = boundary_proto->add_points_vcs();
      vcs_proto->set_x(vcs_data.x);
      vcs_proto->set_y(vcs_data.y);
      vcs_proto->set_z(vcs_data.z);

      auto& wld_data{boundary_data.points_world.at(b_idx)};
      workflow::proto::Vector3f* wld_proto = boundary_proto->add_points_world();
      wld_proto->set_x(wld_data.x);
      wld_proto->set_y(wld_data.y);
      wld_proto->set_z(wld_data.z);

      boundary_proto->add_boundaries_status(
          static_cast<workflow::proto::parking_slot::BoundaryStatus>(
              boundary_data.boundaries_status.at(b_idx)));
      boundary_proto->add_boundaries_height(
          boundary_data.boundaries_height.at(b_idx));
      boundary_proto->add_boundaries_fix_status(
          static_cast<workflow::proto::parking_slot::BoundaryFixStatus>(
              boundary_data.boundaries_fix_status.at(b_idx)));

      {
        auto& point_ss_data{boundary_data.points_delta_ss.at(b_idx)};
        workflow::proto::Vector3f* point_ss_proto =
            boundary_proto->add_points_delta_ss();
        point_ss_proto->set_x(point_ss_data.x);
        point_ss_proto->set_y(point_ss_data.y);
        point_ss_proto->set_z(point_ss_data.z);
      }
    }
  }  // end boundary

  slot_proto->set_type(
      static_cast<workflow::proto::parking_slot::ParkingType>(slot_data.type));
  slot_proto->set_occupancy_status(
      static_cast<workflow::proto::parking_slot::OccupancyStatus>(
          slot_data.occupancy_status));
  slot_proto->set_parking_source(
      static_cast<workflow::proto::parking_slot::ParkingSource>(
          slot_data.parking_source));
  slot_proto->set_stage(
      static_cast<workflow::proto::parking_slot::ParkingStage>(
          slot_data.stage));
  slot_proto->set_parking_release_mode(
      static_cast<workflow::proto::parking_slot::ParkingReleaseMode>(
          slot_data.parking_release_mode));
  slot_proto->set_occupancy_obs_type(
      static_cast<workflow::proto::parking_slot::OccupyObjType>(
          slot_data.occupancy_obs_type));

  slot_proto->set_str_id(slot_data.str_id.data(), slot_data.num_str_id);

  return 0;
}

int ParkingSlotDeserialize(
    const workflow::proto::parking_slot::ParkingSlotInfo& slot_proto,
    ParkingSlotInfo& slot_data) {
  slot_data.id = slot_proto.id();
  slot_data.valid = slot_proto.valid();
  slot_data.conf = slot_proto.conf();
  slot_data.num_polygon_points = slot_proto.polygon_points_size();
  for (std::int32_t pp_idx = 0; pp_idx < slot_proto.polygon_points_size();
       pp_idx++) {
    auto& corner_info_data{slot_data.polygon_points.at(pp_idx)};
    auto& corner_info_proto{slot_proto.polygon_points(pp_idx)};
    corner_info_data.point_vcs.x = corner_info_proto.point_vcs().x();
    corner_info_data.point_vcs.y = corner_info_proto.point_vcs().y();
    corner_info_data.point_vcs.z = corner_info_proto.point_vcs().z();
    corner_info_data.point_world.x = corner_info_proto.point_world().x();
    corner_info_data.point_world.y = corner_info_proto.point_world().y();
    corner_info_data.point_world.z = corner_info_proto.point_world().z();
    corner_info_data.point_cov.x = corner_info_proto.point_cov().x();
    corner_info_data.point_cov.y = corner_info_proto.point_cov().y();
    corner_info_data.point_cov.z = corner_info_proto.point_cov().z();
    corner_info_data.valid = corner_info_proto.valid();
  }

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
    for (std::int32_t vcs_idx = 0; vcs_idx < pose_proto.corners_vcs_size();
         vcs_idx++) {
      pose_data.corners_vcs.at(vcs_idx).x = pose_proto.corners_vcs(vcs_idx).x();
      pose_data.corners_vcs.at(vcs_idx).y = pose_proto.corners_vcs(vcs_idx).y();
      pose_data.corners_vcs.at(vcs_idx).z = pose_proto.corners_vcs(vcs_idx).z();
    }

    for (std::int32_t wld_idx = 0; wld_idx < pose_proto.corners_world_size();
         wld_idx++) {
      pose_data.corners_world.at(wld_idx).x =
          pose_proto.corners_world(wld_idx).x();
      pose_data.corners_world.at(wld_idx).y =
          pose_proto.corners_world(wld_idx).y();
      pose_data.corners_world.at(wld_idx).z =
          pose_proto.corners_world(wld_idx).z();
    }
  }  // end pose

  if (slot_proto.has_parking_rod()) {
    auto& rod_data{slot_data.parking_rod};
    auto& rod_proto{slot_proto.parking_rod()};

    rod_data.num_end_points = rod_proto.end_points_vcs_size();
    for (std::int32_t point_vcs_idx = 0;
         point_vcs_idx < rod_proto.end_points_vcs_size(); point_vcs_idx++) {
      rod_data.end_points_vcs.at(point_vcs_idx).x =
          rod_proto.end_points_vcs(point_vcs_idx).x();
      rod_data.end_points_vcs.at(point_vcs_idx).y =
          rod_proto.end_points_vcs(point_vcs_idx).y();
      rod_data.end_points_vcs.at(point_vcs_idx).z =
          rod_proto.end_points_vcs(point_vcs_idx).z();
    }

    for (std::int32_t point_world_idx = 0;
         point_world_idx < rod_proto.end_points_world_size();
         point_world_idx++) {
      rod_data.end_points_world.at(point_world_idx).x =
          rod_proto.end_points_world(point_world_idx).x();
      rod_data.end_points_world.at(point_world_idx).y =
          rod_proto.end_points_world(point_world_idx).y();
      rod_data.end_points_world.at(point_world_idx).z =
          rod_proto.end_points_world(point_world_idx).z();
    }

    rod_data.valid = rod_proto.valid();
  }  // end parking_rod

  if (slot_proto.has_parking_lock_info()) {
    auto& park_lock_data{slot_data.parking_lock_info};
    auto& park_lock_proto{slot_proto.parking_lock_info()};

    park_lock_data.point_vcs.x = park_lock_proto.point_vcs().x();
    park_lock_data.point_vcs.y = park_lock_proto.point_vcs().y();
    park_lock_data.point_vcs.z = park_lock_proto.point_vcs().z();

    park_lock_data.point_world.x = park_lock_proto.point_world().x();
    park_lock_data.point_world.y = park_lock_proto.point_world().y();
    park_lock_data.point_world.z = park_lock_proto.point_world().z();

    park_lock_data.parking_status =
        static_cast<ParkingLockStatus>(park_lock_proto.parking_status());
    park_lock_data.valid = park_lock_proto.valid();
  }  // end parking_lock_info

  if (slot_proto.has_boundary()) {
    auto& boundary_data{slot_data.boundary};
    auto& boundary_proto{slot_proto.boundary()};

    boundary_data.num_boundary_points = boundary_proto.points_vcs_size();
    for (std::int32_t vcs_idx = 0; vcs_idx < boundary_proto.points_vcs_size();
         vcs_idx++) {
      boundary_data.points_vcs.at(vcs_idx).x =
          boundary_proto.points_vcs(vcs_idx).x();
      boundary_data.points_vcs.at(vcs_idx).y =
          boundary_proto.points_vcs(vcs_idx).y();
      boundary_data.points_vcs.at(vcs_idx).z =
          boundary_proto.points_vcs(vcs_idx).z();
    }

    for (std::int32_t world_idx = 0;
         world_idx < boundary_proto.points_world_size(); world_idx++) {
      boundary_data.points_world.at(world_idx).x =
          boundary_proto.points_world(world_idx).x();
      boundary_data.points_world.at(world_idx).y =
          boundary_proto.points_world(world_idx).y();
      boundary_data.points_world.at(world_idx).z =
          boundary_proto.points_world(world_idx).z();
    }

    for (std::int32_t sts_idx = 0;
         sts_idx < boundary_proto.boundaries_status_size(); sts_idx++) {
      boundary_data.boundaries_status.at(sts_idx) = static_cast<BoundaryStatus>(
          boundary_proto.boundaries_status(sts_idx));
    }

    for (std::int32_t hegt_idx = 0;
         hegt_idx < boundary_proto.boundaries_height_size(); hegt_idx++) {
      boundary_data.boundaries_height.at(hegt_idx) =
          boundary_proto.boundaries_height(hegt_idx);
    }

    for (std::int32_t fix_idx = 0;
         fix_idx < boundary_proto.boundaries_fix_status_size(); fix_idx++) {
      boundary_data.boundaries_fix_status.at(fix_idx) =
          static_cast<BoundaryFixStatus>(
              boundary_proto.boundaries_fix_status(fix_idx));
    }

    for (std::int32_t ss_idx = 0;
         ss_idx < boundary_proto.points_delta_ss_size(); ss_idx++) {
      auto& point_data{boundary_data.points_delta_ss.at(ss_idx)};
      auto& point_proto{boundary_proto.points_delta_ss(ss_idx)};
      point_data.x = point_proto.x();
      point_data.y = point_proto.y();
      point_data.z = point_proto.z();
    }
  }  // end boundary

  slot_data.type = static_cast<ParkingType>(slot_proto.type());
  slot_data.occupancy_status =
      static_cast<OccupancyStatus>(slot_proto.occupancy_status());
  slot_data.parking_source =
      static_cast<ParkingSource>(slot_proto.parking_source());
  slot_data.stage = static_cast<ParkingStage>(slot_proto.stage());
  slot_data.parking_release_mode =
      static_cast<ParkingReleaseMode>(slot_proto.parking_release_mode());
  slot_data.occupancy_obs_type =
      static_cast<OccupyObjType>(slot_proto.occupancy_obs_type());

  auto& str_value{slot_proto.str_id()};
  memcpy(slot_data.str_id.data(), str_value.c_str(), str_value.length());
  slot_data.num_str_id = str_value.length();

  return 0;
}

}  // namespace autodrive
