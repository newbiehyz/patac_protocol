//
#include "interface/parking/serializer/fusion_system_serializer.h"

#include <utils/exception.h>

#include "interface/parking/fusion_system_data.h"
#include "protocol/fusion_system.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/parking_slot_serializer.h"

namespace autodrive {

int FusionSystemSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  auto& fusion_system_data{data_msg->GetFusionSystem()};
  proto_msg->set_lock_id_type(static_cast<workflow::proto::LockIdType>(
      fusion_system_data.lock_id_type));
  proto_msg->set_lock_id(fusion_system_data.lock_id);

  proto_msg->set_enable_backstage_searching(
      fusion_system_data.enable_backstage_searching);
  proto_msg->set_enable_normal_searching(
      fusion_system_data.enable_normal_searching);
  proto_msg->set_enable_peb_prkgperception(
      fusion_system_data.enable_PEB_PrkgPerception);

  proto_msg->set_mirr_fold_sts_at_driver(
      static_cast<workflow::proto::MirrFoldStsTyp>(
          fusion_system_data.mirr_fold_sts_at_driver));
  proto_msg->set_mirr_fold_sts_at_pass(
      static_cast<workflow::proto::MirrFoldStsTyp>(
          fusion_system_data.mirr_fold_sts_at_pass));

  autodrive::workflow::proto::ApaStateInfo* fusion_system_apa_proto{
      proto_msg->mutable_apa_states()};
  fusion_system_apa_proto->set_state_type(
      static_cast<workflow::proto::ApaStateDetailType>(
          fusion_system_data.apa_states.state_type));
  fusion_system_apa_proto->set_func_stage(
      static_cast<workflow::proto::ApaFunctionStage>(
          fusion_system_data.apa_states.func_stage));
  proto_msg->set_vpa_state_detail(
      static_cast<workflow::proto::VpaStateDetailType>(
          fusion_system_data.vpa_state_detail_type));
  proto_msg->set_perception_ctrl_status(
      static_cast<workflow::proto::PerceptionCtrlStatus>(
          fusion_system_data.perception_ctrl_status));
  proto_msg->set_parking_out_dir(static_cast<workflow::proto::ParkingOutDir>(
      fusion_system_data.parking_out_dir));
  proto_msg->set_work_mode_type(static_cast<workflow::proto::WorkModeType>(
      fusion_system_data.work_mode_type));

  proto_msg->mutable_usr_sel_ps()->set_user_sel_parking_slot_received(
      fusion_system_data.usr_sel_ps.user_sel_parking_slot_received);
  ParkingSlotSerialize(
      fusion_system_data.usr_sel_ps.usersel_parking_slot,
      proto_msg->mutable_usr_sel_ps()->mutable_usersel_parking_slot());

  return 0;
}

int FusionSystemSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& fusion_system_data{data_msg->MutableFusionSystem()};
  fusion_system_data.lock_id = proto_msg->lock_id();
  fusion_system_data.lock_id_type =
      static_cast<LockIdType>(proto_msg->lock_id_type());

  fusion_system_data.enable_backstage_searching =
      proto_msg->enable_backstage_searching();
  fusion_system_data.enable_normal_searching =
      proto_msg->enable_normal_searching();
  fusion_system_data.enable_PEB_PrkgPerception =
      proto_msg->enable_peb_prkgperception();
  fusion_system_data.mirr_fold_sts_at_driver =
      static_cast<MirrFoldStsTyp>(proto_msg->mirr_fold_sts_at_driver());
  fusion_system_data.mirr_fold_sts_at_pass =
      static_cast<MirrFoldStsTyp>(proto_msg->mirr_fold_sts_at_pass());

  auto& fusion_system_apa_proto{proto_msg->apa_states()};
  fusion_system_data.apa_states.func_stage =
      static_cast<ApaFunctionStage>(fusion_system_apa_proto.func_stage());
  fusion_system_data.apa_states.state_type =
      static_cast<ApaStateDetailType>(fusion_system_apa_proto.state_type());

  fusion_system_data.parking_out_dir =
      static_cast<FusionSystemData::ParkingOutDir>(
          proto_msg->parking_out_dir());
  fusion_system_data.work_mode_type =
      static_cast<FusionSystemData::WorkModeType>(proto_msg->work_mode_type());

  fusion_system_data.vpa_state_detail_type =
      static_cast<VpaStateDetailType>(proto_msg->vpa_state_detail());
  fusion_system_data.perception_ctrl_status =
      static_cast<FusionSystemData::PerceptionCtrlStatus>(proto_msg->perception_ctrl_status());
  fusion_system_data.usr_sel_ps.user_sel_parking_slot_received =
      proto_msg->usr_sel_ps().user_sel_parking_slot_received();
  ParkingSlotDeserialize(proto_msg->usr_sel_ps().usersel_parking_slot(),
                         fusion_system_data.usr_sel_ps.usersel_parking_slot);

  return 0;
}

}  // namespace autodrive