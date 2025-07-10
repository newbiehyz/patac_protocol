//
#include "interface/parking/serializer/ipm_multitask_serializer.h"

#include <utils/exception.h>

#include "interface/parking/ipm_multitask_data.h"
#include "protocol/ipm_multitask.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int IpmMultitaskSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  std::uint8_t num_slots_raw = data_msg->GetSlotsNum();
  for (std::uint8_t slot_idx = 0; slot_idx < num_slots_raw; slot_idx++) {
    const auto& slot_raw = data_msg->GetSlotsRaw().at(slot_idx);
    auto* slot_raw_proto = proto_msg->add_slots_raw();
    for (const auto& slot_point : slot_raw.slot_points) {
      auto* slot_point_proto = slot_raw_proto->add_slot_points();
      auto* point = slot_point_proto->mutable_point();
      point->set_x(slot_point.point.x);
      point->set_y(slot_point.point.y);
      slot_point_proto->set_score(slot_point.score);
      slot_point_proto->set_point_type(
          static_cast<workflow::proto::ParkingSlotPointType>(
              slot_point.point_type));
      slot_point_proto->set_visibility(slot_point.visibility);
      slot_point_proto->set_point_orientation_x(slot_point.point_orientation_x);
      slot_point_proto->set_point_orientation_y(slot_point.point_orientation_y);
    }
    slot_raw_proto->set_score(slot_raw.score);
    slot_raw_proto->set_corner_match_flag(slot_raw.corner_match_flag);
    slot_raw_proto->set_occupancy(slot_raw.occupancy);
    slot_raw_proto->set_slot_type(
        static_cast<workflow::proto::ParkingSlotType>(slot_raw.slot_type));
    slot_raw_proto->set_slot_orientation_x(slot_raw.slot_orientation_x);
    slot_raw_proto->set_slot_orientation_y(slot_raw.slot_orientation_y);
  }

  std::uint8_t num_wheel_gdp = data_msg->GetWheelGDPNum();
  for (std::uint8_t gdp_idx = 0; gdp_idx < num_wheel_gdp; gdp_idx++) {
    const auto& gdp = data_msg->GetIpmWheelGroundPoints().at(gdp_idx);
    auto* gdb_proto = proto_msg->add_ipm_wheel_ground_points();
    gdb_proto->set_score(gdp.score);
    auto* point_proto = gdb_proto->mutable_point();
    point_proto->set_x(gdp.point.x);
    point_proto->set_y(gdp.point.y);
  }

  proto_msg->set_test(data_msg->GetTest());


  return 0;
}

int IpmMultitaskSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto& slot_raw_datas{data_msg->MutableSlotsRaw()};
  data_msg->SetSlotsNum(static_cast<std::uint8_t>(proto_msg->slots_raw_size()));
  for (std::int32_t slot_idx = 0; slot_idx < proto_msg->slots_raw_size(); ++slot_idx) {
    auto& slot_raw = slot_raw_datas.at(slot_idx);
    auto& slot_raw_proto = proto_msg->slots_raw(slot_idx);
    for (std::int32_t slot_point_idx = 0;
         slot_point_idx < slot_raw_proto.slot_points_size(); ++slot_point_idx) {
      auto& slot_point = slot_raw.slot_points.at(slot_point_idx);
      const auto& slot_point_proto = slot_raw_proto.slot_points(slot_point_idx);
      slot_point.point.x = slot_point_proto.point().x();
      slot_point.point.y = slot_point_proto.point().y();
      slot_point.score = slot_point_proto.score();
      slot_point.point_type =
          static_cast<IpmMultitaskData::ParkingSlotPointType>(
              slot_point_proto.point_type());
      slot_point.visibility = slot_point_proto.visibility();
      slot_point.point_orientation_x = slot_point_proto.point_orientation_x();
      slot_point.point_orientation_y = slot_point_proto.point_orientation_y();
    }
    slot_raw.score = slot_raw_proto.score();
    slot_raw.occupancy = slot_raw_proto.occupancy();
    slot_raw.corner_match_flag = slot_raw_proto.corner_match_flag();
    slot_raw.slot_type = static_cast<IpmMultitaskData::ParkingSlotType>(
        slot_raw_proto.slot_type());
    slot_raw.slot_orientation_x = slot_raw_proto.slot_orientation_x();
    slot_raw.slot_orientation_y = slot_raw_proto.slot_orientation_y();
  }

  auto& gdp_data{data_msg->MutableIpmWheelGroundPoints()};
  data_msg->SetWheelGDPNum(static_cast<std::uint8_t>(proto_msg->ipm_wheel_ground_points_size()));
  for (std::int32_t gdp_idx = 0; gdp_idx < proto_msg->ipm_wheel_ground_points_size(); ++gdp_idx) {
    auto& gdp = gdp_data.at(gdp_idx);
    const auto& gdp_proto = proto_msg->ipm_wheel_ground_points(gdp_idx);
    gdp.score = gdp_proto.score();
    gdp.point.x = gdp_proto.point().x();
    gdp.point.y = gdp_proto.point().y();
  }

  data_msg->SetTest(proto_msg->test());

  return 0;
}

}  // namespace autodrive