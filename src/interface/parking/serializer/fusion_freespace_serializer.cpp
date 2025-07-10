//
#include "interface/parking/serializer/fusion_freespace_serializer.h"

#include <utils/exception.h>

#include "interface/parking/fusion_freespace_data.h"
#include "protocol/fusion_freespace.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int FusionFreespaceSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& freespace_data{data_msg->GetFusionFreespace()};

  workflow::proto::FreespaceFusion* freespace_proto = proto_msg->mutable_fusion_freespace();
  freespace_proto->set_timestamp(freespace_data.timestamp);
  for (std::uint32_t fp_idx = 0; fp_idx < freespace_data.num_freespace_points; ++fp_idx) {
    auto& space_point_data{freespace_data.freespace_points.at(fp_idx)};
    workflow::proto::FusionFreespacePoint* space_point_proto =
        freespace_proto->add_freespace_points();
    space_point_proto->set_conf(space_point_data.conf);
    space_point_proto->set_angle(space_point_data.angle);
    space_point_proto->set_ray_length(space_point_data.ray_length);
    space_point_proto->set_origin_label(space_point_data.origin_label);
    space_point_proto->set_label(static_cast<workflow::proto::FreespacePointLabel>(
        space_point_data.label));
    space_point_proto->mutable_pt_vcs()->set_x(space_point_data.pt_vcs.x);
    space_point_proto->mutable_pt_vcs()->set_y(space_point_data.pt_vcs.y);
    space_point_proto->mutable_vel_vcs()->set_x(space_point_data.vel_vcs.x);
    space_point_proto->mutable_vel_vcs()->set_y(space_point_data.vel_vcs.y);
    space_point_proto->mutable_pt_world()->set_x(space_point_data.pt_world.x);
    space_point_proto->mutable_pt_world()->set_y(space_point_data.pt_world.y);
    space_point_proto->mutable_vel_world()->set_x(space_point_data.vel_world.x);
    space_point_proto->mutable_vel_world()->set_y(space_point_data.vel_world.y);
  }
  return 0;
}

int FusionFreespaceSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& free_space_data{data_msg->MutableFusionFreespace()};
  auto const& free_space_proto{proto_msg->fusion_freespace()};
  free_space_data.timestamp = free_space_proto.timestamp();
  free_space_data.num_freespace_points = free_space_proto.freespace_points_size();
  if (free_space_data.num_freespace_points > FusionFreespaceData::kMaxFreespacePointsSize) {
    throw OutOfResourceException("fusion_freespace_data point num is out of range");
  }
  for (std::uint32_t fs_idx = 0; fs_idx < free_space_data.num_freespace_points; ++fs_idx) {
    auto& space_point_data{free_space_data.freespace_points.at(fs_idx)};
    auto& space_point_proto{free_space_proto.freespace_points(fs_idx)};
    space_point_data.conf = space_point_proto.conf();
    space_point_data.angle = space_point_proto.angle();
    space_point_data.ray_length = space_point_proto.ray_length();
    space_point_data.origin_label = space_point_proto.origin_label();
    space_point_data.label = static_cast<autodrive::FusionFreespaceData::FreespacePointLabel>(
        space_point_proto.label());
    space_point_data.pt_vcs.x = space_point_proto.pt_vcs().x();
    space_point_data.pt_vcs.y = space_point_proto.pt_vcs().y();
    space_point_data.vel_vcs.x = space_point_proto.vel_vcs().x();
    space_point_data.vel_vcs.y = space_point_proto.vel_vcs().y();
    space_point_data.pt_world.x = space_point_proto.pt_world().x();
    space_point_data.pt_world.y = space_point_proto.pt_world().y();
    space_point_data.vel_world.x = space_point_proto.vel_world().x();
    space_point_data.vel_world.y = space_point_proto.vel_world().y();
  }
  return 0;
}

}  // namespace autodrive