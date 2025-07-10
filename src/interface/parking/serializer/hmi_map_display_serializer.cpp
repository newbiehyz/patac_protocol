//

#include "interface/parking/serializer/hmi_map_display_serializer.h"

#include "interface/parking/hmi_map_display_data.h"
#include "protocol/hmi_map_display.pb.h"
#include "protocol/map_display.pb.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>

namespace autodrive {

int HmdMapCommonHeaderSerialize(const HmiMapDisplayData::MapCommonHeader& map_header_data, 
    workflow::proto::HmdMapCommonHeader* map_header_proto) {

  map_header_proto->set_map_id(map_header_data.map_id);
  map_header_proto->set_map_type(
      static_cast<workflow::proto::MapDataType>(map_header_data.map_type));
  map_header_proto->set_utc_time(map_header_data.utc_time);
  map_header_proto->set_latitude(map_header_data.latitude);
  map_header_proto->set_longitude(map_header_data.longitude);
  map_header_proto->set_map_total_height(map_header_data.map_total_height);
  map_header_proto->set_map_total_floor(map_header_data.map_total_floor);
  map_header_proto->set_num_glb_trajectory_point(
    map_header_data.num_glb_trajectory_point);
  map_header_proto->set_slot_parking_times(map_header_data.slot_parking_times);
  return 0;
}

int HmdMapCommonHeaderDeserialize(const workflow::proto::HmdMapCommonHeader& map_header_proto,
    HmiMapDisplayData::MapCommonHeader& map_header_data) {
  map_header_data.map_id = map_header_proto.map_id();
  map_header_data.map_type =
      static_cast<HmiMapDisplayData::MapDataType>(map_header_proto.map_type());
  map_header_data.utc_time = map_header_proto.utc_time();
  map_header_data.latitude = map_header_proto.latitude();
  map_header_data.longitude = map_header_proto.longitude();
  map_header_data.num_glb_trajectory_point =
      map_header_proto.num_glb_trajectory_point();
  map_header_data.map_total_height = map_header_proto.map_total_height();
  map_header_data.map_total_floor = map_header_proto.map_total_floor();
  map_header_data.slot_parking_times = map_header_proto.slot_parking_times();
  return 0;  
}



int HmiMapDisplaySerializer::Serialize(const message::SerializeMsg *data_ptr,
                                       message::DeserializeMsg *msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());


  auto map_display_proto = proto_msg->mutable_map_display();
  auto &map_display_data{data_msg->GetMapDisplayData()};
  {
    auto *map_info_proto =
        map_display_proto->mutable_map_info();
    auto &map_info_data{map_display_data.map_info};
    {
      auto &header_data{map_info_data.header};
      auto *header_proto =
          map_info_proto->mutable_header();
      header_proto->set_map_id(header_data.map_id);
      header_proto->set_map_type(
          static_cast<workflow::proto::MapDataType>(header_data.map_type));
      header_proto->set_utc_time(header_data.utc_time);
      header_proto->set_latitude(header_data.latitude);
      header_proto->set_longitude(header_data.longitude);
      header_proto->set_num_glb_trajectory_point(header_data.num_glb_trajectory_point);
      header_proto->set_map_total_height(header_data.map_total_height);
      header_proto->set_map_total_floor(header_data.map_total_floor);
      header_proto->set_slot_parking_times(header_data.slot_parking_times);
    } // end header

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
      } // end for

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
    } // end global_trajectory

    {
      auto &gpps_data{map_info_data.global_percep_parking_slots};
      auto *gpps_proto =
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
        } // end slot_points
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
      } // end glb_prkg_slots
    }   // end global_percep_parking_slots


    {
      auto& grmm_data{map_info_data.global_road_mark_map};
      auto grmm_proto =
          map_info_proto->mutable_global_road_mark_map();
      for (std::uint32_t grm_idx = 0; grm_idx < grmm_data.num_glb_road_mak_map;
          grm_idx++) {
        auto& grm_data{grmm_data.glb_road_mak_map.at(grm_idx)};
        workflow::proto::GlbRoadMark* grm_proto =
            grmm_proto->add_glb_road_mak_map();

        for (std::uint32_t border_idx = 0;
            border_idx < grm_data.num_border_points; ++border_idx) {
          auto& border_data{grm_data.border.at(border_idx)};
          workflow::proto::Vector2f* border_proto = grm_proto->add_border();
          border_proto->set_x(border_data.x);
          border_proto->set_y(border_data.y);
        }
        grm_proto->mutable_position()->set_x(grm_data.position.x);
        grm_proto->mutable_position()->set_y(grm_data.position.y);
        grm_proto->mutable_position()->set_z(grm_data.position.z);
        grm_proto->set_odom_heading(grm_data.odom_heading);
        grm_proto->set_roadmark_id(grm_data.roadmark_id);
        grm_proto->set_roadmark_type(
            static_cast<workflow::proto::RoadMarkType>(grm_data.roadmark_type));
        grm_proto->set_floor(grm_data.floor);
      }  // end glb_road_mak_map
    } // end global_road_mark_map



    {
      auto &gpo_data{map_info_data.global_perception_obstacle};
      auto *gpo_proto =
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
      } // end glb_prcp_obstacle
    }   // end global_perception_obstacle

    {
      map_info_proto->set_map_distance(map_info_data.map_distance);
    } // end map_distance

    {
      map_info_proto->set_rest_map_distance(map_info_data.rest_map_distance);
    } // end rest_map_distance

    {
      map_info_proto->set_num_speed_bumps(map_info_data.num_speed_bumps);
    } // end num_speed_bumps

    map_display_proto->set_update_map_info(map_display_data.update_map_info);

    map_display_proto->set_num_maps(map_display_data.num_maps);
    for (std::uint8_t map_idx = 0; map_idx < map_display_data.num_maps;
         map_idx++) {
      auto &map_header_data{map_display_data.all_map_headers.at(map_idx)};
      auto *map_header_proto =
          map_display_proto->add_all_map_headers();
      HmdMapCommonHeaderSerialize(map_header_data, map_header_proto);
    } // end all_map_headers - for
  }
  return 0;
}

int HmiMapDisplaySerializer::Deserialize(
    const message::DeserializeMsg *msg_ptr,
    message::SerializeMsg *data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto &map_display_data{data_msg->MutableMapDisplayData()};
  auto map_display_proto = &proto_msg->map_display();


  {
    if (map_display_proto->has_map_info()) {
      auto &map_info_data{map_display_data.map_info};
      auto &map_info_proto{map_display_proto->map_info()};
      if (map_info_proto.has_header()) {
        auto &header_data{map_info_data.header};
        auto &header_proto{map_info_proto.header()};
        header_data.map_id = header_proto.map_id();
        header_data.map_type =
            static_cast<HmiMapDisplayData::MapDataType>(header_proto.map_type());
        header_data.utc_time = header_proto.utc_time();
        header_data.latitude = header_proto.latitude();
        header_data.longitude = header_proto.longitude();
        header_data.num_glb_trajectory_point =
            header_proto.num_glb_trajectory_point();
        header_data.map_total_height = header_proto.map_total_height();
        header_data.map_total_floor = header_proto.map_total_floor();
        header_data.slot_parking_times = header_proto.slot_parking_times();
      } // end header
      
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
        } // end for
        gt_data.start_point.x = gt_proto.start_point().x();
        gt_data.start_point.y = gt_proto.start_point().y();
        gt_data.start_point.z = gt_proto.start_point().z();

        gt_data.end_point.x = gt_proto.end_point().x();
        gt_data.end_point.y = gt_proto.end_point().y();
        gt_data.end_point.z = gt_proto.end_point().z();
      } // end global_trajectory

      if (map_info_proto.has_global_road_mark_map()) {
        auto& road_data{map_info_data.global_road_mark_map};
        auto& road_proto{map_info_proto.global_road_mark_map()};
        road_data.num_glb_road_mak_map = road_proto.glb_road_mak_map_size();
        for (std::uint32_t r_idx = 0; r_idx < road_data.num_glb_road_mak_map;
            r_idx++) {
          auto& mark_map_data{road_data.glb_road_mak_map.at(r_idx)};
          auto& mark_map_proto{road_proto.glb_road_mak_map(r_idx)};
          mark_map_data.num_border_points = mark_map_proto.border_size();
          for (std::uint32_t border_idx = 0;
              border_idx < mark_map_data.num_border_points; ++border_idx) {
            auto& border_data{mark_map_data.border.at(border_idx)};
            auto& border_msg{mark_map_proto.border(border_idx)};
            border_data.x = border_msg.x();
            border_data.y = border_msg.y();
          }
          mark_map_data.position.x = mark_map_proto.position().x();
          mark_map_data.position.y = mark_map_proto.position().y();
          mark_map_data.position.z = mark_map_proto.position().z();
          mark_map_data.odom_heading = mark_map_proto.odom_heading();
          mark_map_data.roadmark_id = mark_map_proto.roadmark_id();
          mark_map_data.roadmark_type = static_cast<HmiMapDisplayData::RoadMarkType>(
              mark_map_proto.roadmark_type());
          mark_map_data.floor = mark_map_proto.floor();
        }  // end glb_road_mak_map
      }  // end global_road_mark_map

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
          slot_data.slot_status = static_cast<HmiMapDisplayData::SlotStatus>(
              slot_proto.slot_status());
          slot_data.slot_type =
              static_cast<HmiMapDisplayData::SlotType>(slot_proto.slot_type());
          slot_data.is_target_slot = slot_proto.is_target_slot();

          slot_data.num_slot_points = slot_proto.slot_points_size();
          for (std::uint32_t sp_idx = 0; sp_idx < slot_data.num_slot_points;
                sp_idx++) {
            auto &slot_point_data{slot_data.slot_points.at(sp_idx)};
            auto &slot_point_proto{slot_proto.slot_points(sp_idx)};
            slot_point_data.x = slot_point_proto.x();
            slot_point_data.y = slot_point_proto.y();
            slot_point_data.z = slot_point_proto.z();
          } // end slot_points
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
        } // end glb_prkg_slots
      }   // end global_percep_parking_slots
      
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
              static_cast<HmiMapDisplayData::Map2HmiObstacleType>(
                  obs_proto.obstacle_type());
          obs_data.danger_level = static_cast<HmiMapDisplayData::DangerLevel>(
              obs_proto.danger_level());
          obs_data.target_id = obs_proto.target_id();
          obs_data.obj_status =
              static_cast<HmiMapDisplayData::ObjStatus>(obs_proto.obj_status());
          obs_data.floor = obs_proto.floor();
        } // end glb_prcp_obstacle
      }   // end global_perception_obstacle

      {
        map_info_data.map_distance = map_info_proto.map_distance();
      } // end map_distance

      {
        map_info_data.rest_map_distance = map_info_proto.rest_map_distance();
      } // end rest_map_distance

      {
        map_info_data.num_speed_bumps = map_info_proto.num_speed_bumps();
      } // end num_
    }   // end map_info

    map_display_data.update_map_info = map_display_proto->update_map_info();

    map_display_data.num_maps =
        static_cast<std::int8_t>(map_display_proto->all_map_headers_size());
    for (std::int8_t idx = 0; idx < map_display_data.num_maps; idx++) {
      auto &map_header_data{map_display_data.all_map_headers.at(idx)};
      auto &map_header_proto{map_display_proto->all_map_headers(idx)};
      HmdMapCommonHeaderDeserialize(map_header_proto,
                                            map_header_data);
    }
  }

  return 0;
}
} // namespace autodrive
