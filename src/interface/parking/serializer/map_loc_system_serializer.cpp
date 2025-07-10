//
#include "interface/parking/serializer/map_loc_system_serializer.h"

#include <utils/exception.h>

#include "interface/parking/map_loc_system_data.h"
#include "protocol/map_loc_system.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int MapLocSystemSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& map_loc_system_data{data_msg->GetMapLocSystem()};
  proto_msg->set_avp_lp_state(static_cast<workflow::proto::AVP_LPState>(
      map_loc_system_data.avp_lp_state));
  proto_msg->set_control_cmd(static_cast<workflow::proto::ControlCmd>(
      map_loc_system_data.control_cmd));
  proto_msg->set_target_parking_slot_id(
      map_loc_system_data.target_parking_slot_id);
  proto_msg->set_delete_map_id(
      map_loc_system_data.delete_map_id);
  proto_msg->set_select_map_id(
      map_loc_system_data.select_map_id);

  
  autodrive::workflow::proto::ApaStateInfo* maploc_system_apa_proto{
      proto_msg->mutable_apa_states()};
  maploc_system_apa_proto->set_state_type(
      static_cast<workflow::proto::ApaStateDetailType>(
          map_loc_system_data.apa_states.state_type));
  maploc_system_apa_proto->set_func_stage(
      static_cast<workflow::proto::ApaFunctionStage>(
          map_loc_system_data.apa_states.func_stage));
  return 0;
}

int MapLocSystemSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& map_loc_system_data{data_msg->MutableMapLocSystem()};
  map_loc_system_data.avp_lp_state =
      static_cast<MapLocSystemData::AVP_LPState>(proto_msg->avp_lp_state());
  map_loc_system_data.control_cmd =
      static_cast<MapLocSystemData::ControlCmd>(proto_msg->control_cmd());
  map_loc_system_data.target_parking_slot_id =
      proto_msg->target_parking_slot_id();
  map_loc_system_data.delete_map_id =
      proto_msg->delete_map_id();
  map_loc_system_data.select_map_id = 
      proto_msg->select_map_id();

  auto& maploc_system_apa_proto{proto_msg->apa_states()};
  map_loc_system_data.apa_states.func_stage =
      static_cast<ApaFunctionStage>(maploc_system_apa_proto.func_stage());
  map_loc_system_data.apa_states.state_type =
      static_cast<ApaStateDetailType>(maploc_system_apa_proto.state_type());
  return 0;
}

}  // namespace autodrive