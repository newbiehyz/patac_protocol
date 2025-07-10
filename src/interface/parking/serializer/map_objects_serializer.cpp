//
#include "interface/parking/serializer/map_objects_serializer.h"

#include <utils/exception.h>

#include "interface/parking/map_objects_data.h"
#include "protocol/map_objects.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int MapObjectsSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& map_obj_data{data_msg->GetMapObjects()};
  proto_msg->set_conf_scale(map_obj_data.conf_scale);

  for (std::int32_t obj_idx = 0; obj_idx < map_obj_data.num_objects;
       obj_idx++) {
    auto& obj_data{map_obj_data.objects.at(obj_idx)};
    workflow::proto::MapObject* obj_proto = proto_msg->add_objects();
    obj_proto->set_id(obj_data.id);
    obj_proto->set_cam_id(obj_data.cam_id);
    obj_proto->set_time_stamp(obj_data.time_stamp);
    obj_proto->set_conf(obj_data.conf);
    obj_proto->set_type(
        static_cast<workflow::proto::MapObjectType>(obj_data.type));
    obj_proto->set_sub_type(
        static_cast<workflow::proto::MapObjectSubType>(obj_data.sub_type));
    obj_proto->set_id(obj_data.id);

    {
      auto& border_data{obj_data.border};
      workflow::proto::perception_base::Polygon* border_proto =
          obj_proto->mutable_border();
      for (std::uint32_t p_idx = 0; p_idx < border_data.num_points; p_idx++) {
        auto& point_data{border_data.points.at(p_idx)};
        workflow::proto::Vector3f* point_proto =
            border_proto->add_polygon_points();
        point_proto->set_x(point_data.x);
        point_proto->set_y(point_data.y);
        point_proto->set_z(point_data.z);
      }
    }  // end border

    obj_proto->set_life_time(obj_data.life_time);
    obj_proto->set_age(obj_data.age);
    obj_proto->set_str_id(obj_data.str_id.data(), obj_data.num_str_id);

    {
      auto& attr_data{obj_data.attr};
      workflow::proto::MapObjectAttr* attr_proto = obj_proto->mutable_attr();
      attr_proto->set_shape(attr_data.shape);
      attr_proto->set_cycle_time(attr_data.cycle_time);
      attr_proto->set_traversable(attr_data.traversable);
      attr_proto->set_radius(attr_data.radius);
      attr_proto->set_status(attr_data.status);
      attr_proto->set_value(attr_data.value.data(), attr_data.num_value);
      attr_proto->set_dynamic(attr_data.dynamic);

      {
        auto& img_data{attr_data.img_info};
        workflow::proto::perception_base::ImageSpace* img_proto =
            attr_proto->mutable_img_info();
        img_proto->mutable_rect()->set_bottom(img_data.rect.bottom);
        img_proto->mutable_rect()->set_left(img_data.rect.left);
        img_proto->mutable_rect()->set_right(img_data.rect.right);
        img_proto->mutable_rect()->set_top(img_data.rect.top);
      }  // end img_info
    }    // end attr

    {
      auto& position_data{obj_data.position};
      workflow::proto::Vector3f* position_proto = obj_proto->mutable_position();
      position_proto->set_x(position_data.x);
      position_proto->set_y(position_data.y);
      position_proto->set_z(position_data.z);
    }  // end position
  }
  return 0;
}

int MapObjectsSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                      message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& map_obj_data{data_msg->MutableMapObjects()};

  map_obj_data.conf_scale = proto_msg->conf_scale();

  for (std::int32_t obj_idx = 0; obj_idx < proto_msg->objects_size();
       obj_idx++) {
    auto& obj_data{map_obj_data.objects.at(obj_idx)};
    auto& obj_proto{proto_msg->objects(obj_idx)};
    obj_data.id = obj_proto.id();
    obj_data.cam_id = obj_proto.cam_id();
    obj_data.time_stamp = obj_proto.time_stamp();
    obj_data.conf = obj_proto.conf();
    obj_data.type =
        static_cast<MapObjectsData::MapObjectType>(obj_proto.type());
    obj_data.sub_type =
        static_cast<MapObjectsData::MapObjectSubType>(obj_proto.sub_type());

    if (obj_proto.has_border()) {
      auto& border_data{obj_data.border};
      auto& border_proto{obj_proto.border()};
      border_data.num_points = border_proto.polygon_points_size();
      for (std::uint32_t p_idx = 0; p_idx < border_data.num_points; p_idx++) {
        auto& point_data{border_data.points.at(p_idx)};
        auto& point_proto{border_proto.polygon_points(p_idx)};
        point_data.x = point_proto.x();
        point_data.y = point_proto.y();
        point_data.z = point_proto.z();
      }
    }  // end border

    obj_data.life_time = obj_proto.life_time();
    obj_data.age = obj_proto.age();
    auto& str_value{obj_proto.str_id()};
    obj_data.num_str_id = str_value.length();
    memcpy(obj_data.str_id.data(), str_value.c_str(), obj_data.num_str_id);

    if (obj_proto.has_attr()) {
      auto& attr_data{obj_data.attr};
      auto& attr_proto{obj_proto.attr()};
      attr_data.shape = attr_proto.shape();
      attr_data.cycle_time = attr_proto.cycle_time();
      attr_data.traversable = attr_proto.traversable();
      attr_data.height = attr_proto.height();
      attr_data.radius = attr_proto.radius();
      attr_data.status = attr_proto.status();

      auto& attr_value{attr_proto.value()};
      memcpy(attr_data.value.data(), attr_value.c_str(), attr_value.length());

      attr_data.dynamic = attr_proto.dynamic();

      if (attr_proto.has_img_info()) {
        auto& img_data{attr_data.img_info};
        auto& img_proto{attr_proto.img_info()};
        img_data.rect.bottom = img_proto.rect().bottom();
        img_data.rect.left = img_proto.rect().left();
        img_data.rect.right = img_proto.rect().right();
        img_data.rect.top = img_proto.rect().top();
      }  // end img_info
    }    // end attr

    if (obj_proto.has_position()) {
      auto& pos_data{obj_data.position};
      auto& pos_proto{obj_proto.position()};
      pos_data.x = pos_proto.x();
      pos_data.y = pos_proto.y();
      pos_data.z = pos_proto.z();
    }  // end position
  }
  return 0;
}

}  // namespace autodrive