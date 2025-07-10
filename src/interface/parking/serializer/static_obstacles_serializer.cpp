//
#include "interface/parking/serializer/static_obstacles_serializer.h"

#include <utils/exception.h>

#include "interface/parking/static_obstacles_data.h"
#include "protocol/static_obstacles.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int StaticObstaclesSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& static_obsts_data{data_msg->GetStaticObstacles()};
  proto_msg->set_conf_scale(static_obsts_data.conf_scale);
  for (std::int32_t so_idx = 0; so_idx < static_obsts_data.num_static_obstacles;
       so_idx++) {
    auto& obs_data{static_obsts_data.static_obstacles.at(so_idx)};
    workflow::proto::StaticObstacle* obs_proto =
        proto_msg->add_static_obstacles();

    obs_proto->set_id(obs_data.id);
    obs_proto->set_cam_id(obs_data.cam_id);
    obs_proto->set_time_stamp(obs_data.time_stamp);
    obs_proto->set_conf(obs_data.conf);
    obs_proto->set_type(
        static_cast<workflow::proto::StaticObstacleType>(obs_data.type));
    obs_proto->set_sub_type(
        static_cast<workflow::proto::StaticObstacleSubType>(obs_data.sub_type));

    {
      auto& border_data{obs_data.border};
      workflow::proto::perception_base::Polygon* border_proto =
          obs_proto->mutable_border();
      for (std::uint32_t p_idx = 0; p_idx < border_data.num_points; p_idx++) {
        auto* polygon_proto = border_proto->add_polygon_points();
        polygon_proto->set_x(border_data.points.at(p_idx).x);
        polygon_proto->set_y(border_data.points.at(p_idx).y);
        polygon_proto->set_z(border_data.points.at(p_idx).z);
      }
    }  // end border

    obs_proto->set_life_time(obs_data.life_time);
    obs_proto->set_age(obs_data.age);
    obs_proto->set_str_id(obs_data.str_id.data(), obs_data.num_str_id);

    {
      auto& attr_data{obs_data.attr};
      workflow::proto::StaticObstacleAttr* attr_proto =
          obs_proto->mutable_attr();
      attr_proto->set_cycle_time(attr_data.cycle_time);
      attr_proto->set_traversable(attr_data.traversable);
      attr_proto->set_height(attr_data.height);
      attr_proto->set_radius(attr_data.radius);
      attr_proto->set_status(attr_data.status);
      attr_proto->set_value(attr_data.value, attr_data.num_value);
      attr_proto->set_dynamic(attr_data.dynamic);
      attr_proto->set_shape(attr_data.shape);
      {
        auto& image_data{attr_data.img_info};
        attr_proto->mutable_img_info()->mutable_rect()->set_bottom(
            image_data.rect.bottom);
        attr_proto->mutable_img_info()->mutable_rect()->set_left(
            image_data.rect.left);
        attr_proto->mutable_img_info()->mutable_rect()->set_right(
            image_data.rect.right);
        attr_proto->mutable_img_info()->mutable_rect()->set_top(
            image_data.rect.top);
      }  // end img_info
    }    // end attr

    {
      auto& pos_data{obs_data.position};
      workflow::proto::Vector3f* pos_proto = obs_proto->mutable_position();
      pos_proto->set_x(pos_data.x);
      pos_proto->set_y(pos_data.y);
      pos_proto->set_z(pos_data.z);
    }  // end position
  }    // end static_obstacles & for
  return 0;
}

int StaticObstaclesSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& obsts_data{data_msg->MutableStaticObstacles()};

  obsts_data.conf_scale = proto_msg->conf_scale();
  obsts_data.num_static_obstacles = proto_msg->static_obstacles_size();
  for (std::int32_t obs_idx = 0; obs_idx < obsts_data.num_static_obstacles;
       obs_idx++) {
    auto& obs_data{obsts_data.static_obstacles.at(obs_idx)};
    auto& obs_proto{proto_msg->static_obstacles(obs_idx)};
    obs_data.id = obs_proto.id();
    obs_data.cam_id = obs_proto.cam_id();
    obs_data.time_stamp = obs_proto.time_stamp();
    obs_data.conf = obs_proto.conf();
    obs_data.type =
        static_cast<StaticObstaclesData::StaticObstacleType>(obs_proto.type());
    obs_data.sub_type = static_cast<StaticObstaclesData::StaticObstacleSubType>(
        obs_proto.sub_type());

    if (obs_proto.has_border()) {
      auto& border_data{obs_data.border};
      auto& border_proto{obs_proto.border()};
      border_data.num_points = border_proto.polygon_points_size();
      for (std::uint32_t p_idx = 0; p_idx < border_data.num_points; p_idx++) {
        auto& p_data{border_data.points.at(p_idx)};
        auto& p_proto{border_proto.polygon_points(p_idx)};
        p_data.x = p_proto.x();
        p_data.y = p_proto.y();
        p_data.z = p_proto.z();
      }  // end points - for
    }    // end border

    obs_data.life_time = obs_proto.life_time();
    obs_data.age = obs_proto.age();

    auto& str_value = obs_proto.str_id();
    obs_data.num_str_id = static_cast<std::uint8_t>(str_value.length());
    memcpy(obs_data.str_id.data(), str_value.c_str(), str_value.length());

    if (obs_proto.has_attr()) {
      auto& attr_data{obs_data.attr};
      auto& attr_proto{obs_proto.attr()};
      attr_data.shape = attr_proto.shape();
      attr_data.cycle_time = attr_proto.cycle_time();
      attr_data.traversable = attr_proto.traversable();
      attr_data.height = attr_proto.height();
      attr_data.radius = attr_proto.radius();
      attr_data.status = attr_proto.status();
      auto& value{attr_proto.value()};
      attr_data.num_value = value.length();
      memcpy(attr_data.value, value.c_str(), value.length());
      attr_data.dynamic = attr_proto.dynamic();
      {
        attr_data.img_info.rect.bottom = attr_proto.img_info().rect().bottom();
        attr_data.img_info.rect.left = attr_proto.img_info().rect().left();
        attr_data.img_info.rect.right = attr_proto.img_info().rect().right();
        attr_data.img_info.rect.top = attr_proto.img_info().rect().top();
      }  // end img_info
    }    // end attr

    if (obs_proto.has_position()) {
      obs_data.position.x = obs_proto.position().x();
      obs_data.position.y = obs_proto.position().y();
      obs_data.position.z = obs_proto.position().z();
    }  // end position
  }    // end obstacle-for
  return 0;
}

}  // namespace autodrive