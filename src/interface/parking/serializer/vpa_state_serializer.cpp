//
#include "interface/parking/serializer/vpa_state_serializer.h"

#include <utils/exception.h>

#include "interface/parking/vpa_state_data.h"
#include "protocol/vpa_state.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int VpaStateDataSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& vpa_state_data{data_msg->GetVpaStateData()};

  proto_msg->set_avaliable_map_id(vpa_state_data.avaliable_map_id);
  proto_msg->set_vpa_event(
      static_cast<workflow::proto::VPAEventType>(vpa_state_data.vpa_event));
  proto_msg->set_hpa_state(
      static_cast<workflow::proto::HPAState>(vpa_state_data.hpa_state));

  proto_msg->set_mapping_odd_type(static_cast<workflow::proto::MappingOddType>(
      vpa_state_data.mapping_odd_type));
  proto_msg->set_loc_odd_type(
      static_cast<workflow::proto::LocOddType>(vpa_state_data.loc_odd_type));
  proto_msg->set_map_saving_stage(static_cast<workflow::proto::MapSavingStage>(
      vpa_state_data.map_saving_stage));
  proto_msg->set_local_loc_stage(static_cast<workflow::proto::LocalLocStage>(
      vpa_state_data.local_loc_stage));
  proto_msg->set_error_code(
      static_cast<workflow::proto::ErrorCode>(vpa_state_data.error_code));
  proto_msg->set_map_deleted_stage(
      static_cast<workflow::proto::MapDeletingStage>(
          vpa_state_data.map_deleted_stage));
  proto_msg->set_map_stage(
      static_cast<workflow::proto::MappingStage>(
          vpa_state_data.map_stage)); 
 proto_msg->set_map_optimized_stage(
      static_cast<workflow::proto::MapOptimizeStage>(
          vpa_state_data.map_optimized_stage));
 proto_msg->set_target_parking_slot_id(
          vpa_state_data.target_parking_slot_id);
 proto_msg->set_map_saving_progress(
          vpa_state_data.map_saving_progress);

  return 0;
}

int VpaStateDataSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& vpa_state_data{data_msg->MutableVpaStateData()};

  vpa_state_data.avaliable_map_id = proto_msg->avaliable_map_id();
  vpa_state_data.vpa_event =
      static_cast<VpaStateData::VPAEventType>(proto_msg->vpa_event());
  vpa_state_data.hpa_state =
      static_cast<VpaStateData::HPAState>(proto_msg->hpa_state());
  vpa_state_data.mapping_odd_type =
      static_cast<VpaStateData::MappingOddType>(proto_msg->mapping_odd_type());
  vpa_state_data.loc_odd_type =
      static_cast<VpaStateData::LocOddType>(proto_msg->loc_odd_type());
  vpa_state_data.map_saving_stage =
      static_cast<VpaStateData::MapSavingStage>(proto_msg->map_saving_stage());
  vpa_state_data.local_loc_stage =
      static_cast<VpaStateData::LocalLocStage>(proto_msg->local_loc_stage());
  vpa_state_data.error_code =
      static_cast<VpaStateData::ErrorCode>(proto_msg->error_code());
  vpa_state_data.map_deleted_stage =
      static_cast<VpaStateData::MapDeletingStage>(
          proto_msg->map_deleted_stage());
  vpa_state_data.map_stage =
      static_cast<VpaStateData::MappingStage>(
          proto_msg->map_stage());
  vpa_state_data.map_optimized_stage =
      static_cast<VpaStateData::MapOptimizeStage>(
          proto_msg->map_optimized_stage());
  vpa_state_data.target_parking_slot_id =
            proto_msg->target_parking_slot_id();
  vpa_state_data.map_saving_progress =
            proto_msg->map_saving_progress();

  return 0;
}

}  // namespace autodrive