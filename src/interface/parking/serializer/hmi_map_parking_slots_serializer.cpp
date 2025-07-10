//

#include "interface/parking/serializer/hmi_map_parking_slots_serializer.h"

#include <utils/exception.h>

#include "interface/parking/hmi_map_parking_slots_data.h"
#include "protocol/hmi_map_parking_slots.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {
int HmiMapParkingSlotsSerializer::Serialize(
    const message::SerializeMsg *data_ptr,
    message::DeserializeMsg *msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto &map_parking_slots{data_msg->GetMapParkingSlots()};

  workflow::proto::MapParkingSlots *map_parking_slots_proto =
      proto_msg->mutable_map_parking_slots();
  for (std::uint32_t slot_idx = 0; slot_idx < map_parking_slots.slot_num;
       slot_idx++) {
    auto &all_map_parking_slot{
        map_parking_slots.all_map_parking_slots.at(slot_idx)};
    workflow::proto::MapParkingSlot *map_parking_slot_proto =
        map_parking_slots_proto->add_all_map_parking_slots();
    map_parking_slot_proto->set_slot_id(all_map_parking_slot.slot_id);
    map_parking_slot_proto->set_slot_state(all_map_parking_slot.slot_state);
    map_parking_slot_proto->set_slot_type(all_map_parking_slot.slot_type);
    for (std::uint8_t point_num = 0;
         point_num < autodrive::HmiMapParkingSlotsData::kPointNum;
         point_num++) {
      auto &slot_point{all_map_parking_slot.slot_points.at(point_num)};
      workflow::proto::Vector3f *slot_point_proto =
          map_parking_slot_proto->add_slot_points();
      slot_point_proto->set_x(slot_point.x);
      slot_point_proto->set_y(slot_point.y);
      slot_point_proto->set_z(slot_point.z);
    }
  }
  return 0;
}

int HmiMapParkingSlotsSerializer::Deserialize(
    const message::DeserializeMsg *msg_ptr,
    message::SerializeMsg *data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto &map_parking_slots{data_msg->MutableMapParkingSlots()};
  map_parking_slots.slot_num =
      proto_msg->map_parking_slots().all_map_parking_slots_size();
  if (map_parking_slots.slot_num >
      autodrive::HmiMapParkingSlotsData::kMaxSlotNum) {
    throw OutOfResourceException("all map parking slots num is out of range");
  }
  for (std::uint32_t i = 0; i < map_parking_slots.slot_num; ++i) {
    auto &map_parking_slot{map_parking_slots.all_map_parking_slots.at(i)};
    auto &parking_slots_proto{
        proto_msg->map_parking_slots().all_map_parking_slots(i)};
    map_parking_slot.slot_id = parking_slots_proto.slot_id();
    map_parking_slot.slot_state = parking_slots_proto.slot_state();
    map_parking_slot.slot_type = parking_slots_proto.slot_type();
    for (std::uint8_t point_num = 0;
         point_num < autodrive::HmiMapParkingSlotsData::kPointNum;
         point_num++) {
      map_parking_slot.slot_points.at(point_num).x =
          parking_slots_proto.slot_points(point_num).x();
      map_parking_slot.slot_points.at(point_num).y =
          parking_slots_proto.slot_points(point_num).y();
      map_parking_slot.slot_points.at(point_num).z =
          parking_slots_proto.slot_points(point_num).z();
    }
  }
  return 0;
}
}  // namespace autodrive
