//
#include "interface/parking/serializer/fusion_obstacle_serializer.h"

#include <utils/exception.h>

#include "interface/parking/fusion_obstacle_data.h"
#include "protocol/fusion_obstacle.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int FusionObstacleSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& fusion_obstacle_data{data_msg->GetFusionObstacle()};

  workflow::proto::FusionObstacles* obsts_proto =
      proto_msg->mutable_fusion_obstacles();
  auto& obsts_data{fusion_obstacle_data};
  obsts_proto->set_timestamp(obsts_data.timestamp);
  for (std::uint32_t ol_idx = 0; ol_idx < obsts_data.num_obstacle_list;
       ol_idx++) {
    auto& obst_data{obsts_data.obstacle_list.at(ol_idx)};
    workflow::proto::FusionObstacleOutPut* obst_proto =
        obsts_proto->add_obstacle_list();
    obst_proto->set_age(obst_data.age);
    obst_proto->set_is_updated(obst_data.is_updated);
    obst_proto->set_sensor_source(obst_data.sensor_source);
    obst_proto->set_id(obst_data.id);
    obst_proto->set_vision_id(obst_data.vision_id);
    obst_proto->set_uss_id(obst_data.uss_id);
    obst_proto->set_mono_id(obst_data.mono_id);
    obst_proto->mutable_pos()->set_x(obst_data.pos.x);
    obst_proto->mutable_pos()->set_y(obst_data.pos.y);
    obst_proto->mutable_pos()->set_z(obst_data.pos.z);
    obst_proto->mutable_pos_std()->set_x(obst_data.pos_std.x);
    obst_proto->mutable_pos_std()->set_y(obst_data.pos_std.y);
    obst_proto->mutable_pos_std()->set_z(obst_data.pos_std.z);
    obst_proto->mutable_vel()->set_x(obst_data.vel.x);
    obst_proto->mutable_vel()->set_y(obst_data.vel.y);
    obst_proto->mutable_vel()->set_z(obst_data.vel.z);
    obst_proto->mutable_vel_abs_world()->set_x(obst_data.vel_abs_world.x);
    obst_proto->mutable_vel_abs_world()->set_y(obst_data.vel_abs_world.y);
    obst_proto->mutable_vel_abs_world()->set_z(obst_data.vel_abs_world.z);
    obst_proto->mutable_acc()->set_x(obst_data.acc.x);
    obst_proto->mutable_acc()->set_y(obst_data.acc.y);
    obst_proto->mutable_acc()->set_z(obst_data.acc.z);
    obst_proto->mutable_acc_abs_world()->set_x(obst_data.acc_abs_world.x);
    obst_proto->mutable_acc_abs_world()->set_y(obst_data.acc_abs_world.y);
    obst_proto->mutable_acc_abs_world()->set_z(obst_data.acc_abs_world.z);
    obst_proto->set_heading(obst_data.heading);
    obst_proto->set_angular_speed(obst_data.angular_speed);
    obst_proto->set_obstacle_conf(obst_data.obstacle_conf);
    obst_proto->set_width(obst_data.width);
    obst_proto->set_length(obst_data.length);
    obst_proto->set_height(obst_data.height);
    obst_proto->set_ttc(obst_data.ttc);
    obst_proto->set_type(
        static_cast<workflow::proto::FusionObstacleType>(obst_data.type));
    obst_proto->set_veh_sub_type(
        static_cast<workflow::proto::VehicleSubType>(obst_data.veh_sub_type));
    obst_proto->set_static_sub_type(
        static_cast<workflow::proto::FusionObstacleStaticSubType>(
            obst_data.static_sub_type));
    obst_proto->set_motion_state(
        static_cast<workflow::proto::ObstacleMotionStatus>(
            obst_data.motion_state));
    obst_proto->mutable_uss_pos()->mutable_pos1()->set_x(
        obst_data.uss_pos.pos1.x);
    obst_proto->mutable_uss_pos()->mutable_pos1()->set_y(
        obst_data.uss_pos.pos1.y);
    obst_proto->mutable_uss_pos()->mutable_pos1()->set_z(
        obst_data.uss_pos.pos1.z);
    obst_proto->mutable_uss_pos()->mutable_pos2()->set_x(
        obst_data.uss_pos.pos2.x);
    obst_proto->mutable_uss_pos()->mutable_pos2()->set_y(
        obst_data.uss_pos.pos2.y);
    obst_proto->mutable_uss_pos()->mutable_pos2()->set_z(
        obst_data.uss_pos.pos2.z);
    obst_proto->set_motion_category(
        static_cast<workflow::proto::ObstacleMotionCategory>(
            obst_data.motion_category));
    obst_proto->set_motion_orientation(
        static_cast<workflow::proto::ObstacleMotionOrientation>(
            obst_data.motion_orientation));
  }  // end fusion_obstacles

  return 0;
}

int FusionObstacleSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& fusion_obstacle_data{data_msg->MutableFusionObstacle()};

  if (proto_msg->has_fusion_obstacles()) {
    auto& obst_proto{proto_msg->fusion_obstacles()};
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
      output_data.vel_abs_world.x = output_proto.vel_abs_world().x();
      output_data.vel_abs_world.y = output_proto.vel_abs_world().y();
      output_data.vel_abs_world.z = output_proto.vel_abs_world().z();
      output_data.acc.x = output_proto.acc().x();
      output_data.acc.y = output_proto.acc().y();
      output_data.acc.z = output_proto.acc().z();
      output_data.acc_abs_world.x = output_proto.acc_abs_world().x();
      output_data.acc_abs_world.y = output_proto.acc_abs_world().y();
      output_data.acc_abs_world.z = output_proto.acc_abs_world().z();
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

  return 0;
}

}