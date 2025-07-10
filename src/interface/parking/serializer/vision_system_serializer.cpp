//
#include "interface/parking/serializer/vision_system_serializer.h"

#include <utils/exception.h>

#include "interface/parking/vision_system_data.h"
#include "protocol/vision_system.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/parking_slot_serializer.h"

namespace autodrive {

int VisionSystemSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  auto& vision_system_data{data_msg->GetVisionSystem()};
  proto_msg->set_lock_id_type(static_cast<workflow::proto::LockIdType>(
      vision_system_data.lock_id_type));
  proto_msg->set_parking_in_id(vision_system_data.parking_in_id);
  proto_msg->set_parking_out_id(vision_system_data.parking_out_id);
  proto_msg->set_system_status(static_cast<workflow::proto::VisionSystemStatus>(
      vision_system_data.system_status));

  proto_msg->set_calibration_status(
      static_cast<workflow::proto::VisionCalibrationStatus>(
          vision_system_data.calibration_status));
  proto_msg->set_calib_type(
      static_cast<workflow::proto::VisionCalibrationType>(
          vision_system_data.calib_type));
  proto_msg->set_calib_mode(static_cast<workflow::proto::CalibrationMode>(
      vision_system_data.calib_mode));
  proto_msg->set_calib_pattern(static_cast<workflow::proto::EolCalibPattern>(
      vision_system_data.calib_pattern));

  proto_msg->set_enable_backstage_searching(
      vision_system_data.enable_backstage_searching);
  proto_msg->set_enable_normal_searching(
      vision_system_data.enable_normal_searching);
  proto_msg->set_enable_peb_prkgperception(
      vision_system_data.enable_PEB_PrkgPerception);
  proto_msg->set_mirr_fold_sts_at_driver(
      static_cast<workflow::proto::MirrFoldStsTyp>(
          vision_system_data.mirr_fold_sts_at_driver));
  proto_msg->set_mirr_fold_sts_at_pass(
      static_cast<workflow::proto::MirrFoldStsTyp>(
          vision_system_data.mirr_fold_sts_at_pass));
  proto_msg->set_vpa_state_detail_type(
      static_cast<workflow::proto::VpaStateDetailType>(
          vision_system_data.vpa_state_detail_type));

  autodrive::workflow::proto::ApaStateInfo* vision_system_apa_proto{
      proto_msg->mutable_apa_states()};
  vision_system_apa_proto->set_state_type(
      static_cast<workflow::proto::ApaStateDetailType>(
          vision_system_data.apa_states.state_type));
  vision_system_apa_proto->set_func_stage(
      static_cast<workflow::proto::ApaFunctionStage>(
          vision_system_data.apa_states.func_stage));

  proto_msg->mutable_usr_sel_ps()->set_user_sel_parking_slot_received(
      vision_system_data.usr_sel_ps.user_sel_parking_slot_received);
  ParkingSlotSerialize(
      vision_system_data.usr_sel_ps.usersel_parking_slot,
      proto_msg->mutable_usr_sel_ps()->mutable_usersel_parking_slot());
  proto_msg->set_work_mode_type(static_cast<workflow::proto::WorkModeType>(
      vision_system_data.work_mode_type));
  return 0;
}

int VisionSystemSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto& vision_system_data{data_msg->MutableVisionSystem()};
  vision_system_data.lock_id_type =
      static_cast<LockIdType>(proto_msg->lock_id_type());
  vision_system_data.parking_in_id = proto_msg->parking_in_id();
  vision_system_data.parking_out_id = proto_msg->parking_out_id();
  vision_system_data.system_status =
      static_cast<VisionSystemData::VisionSystemStatus>(
          proto_msg->system_status());

  vision_system_data.calibration_status =
      static_cast<VisionSystemData::CalibrationStatus>(
          proto_msg->calibration_status());
  vision_system_data.calib_type =
      static_cast<VisionSystemData::CalibrationType>(
          proto_msg->calib_type());
  vision_system_data.calib_mode =
      static_cast<VisionSystemData::CalibrationMode>(proto_msg->calib_mode());
  vision_system_data.calib_pattern =
      static_cast<VisionSystemData::EolCalibPattern>(
          proto_msg->calib_pattern());

  vision_system_data.enable_backstage_searching =
      proto_msg->enable_backstage_searching();
  vision_system_data.enable_normal_searching =
      proto_msg->enable_normal_searching();
  vision_system_data.enable_PEB_PrkgPerception =
      proto_msg->enable_peb_prkgperception();
  vision_system_data.mirr_fold_sts_at_driver =
      static_cast<MirrFoldStsTyp>(proto_msg->mirr_fold_sts_at_driver());
  vision_system_data.mirr_fold_sts_at_pass =
      static_cast<MirrFoldStsTyp>(proto_msg->mirr_fold_sts_at_pass());
  vision_system_data.vpa_state_detail_type =
      static_cast<VpaStateDetailType>(proto_msg->vpa_state_detail_type());
  auto& vision_system_apa_proto{proto_msg->apa_states()};
  vision_system_data.apa_states.func_stage =
      static_cast<ApaFunctionStage>(vision_system_apa_proto.func_stage());
  vision_system_data.apa_states.state_type =
      static_cast<ApaStateDetailType>(vision_system_apa_proto.state_type());

  vision_system_data.usr_sel_ps.user_sel_parking_slot_received =
      proto_msg->usr_sel_ps().user_sel_parking_slot_received();
  ParkingSlotDeserialize(proto_msg->usr_sel_ps().usersel_parking_slot(),
                         vision_system_data.usr_sel_ps.usersel_parking_slot);
  vision_system_data.work_mode_type =
      static_cast<VisionSystemData::WorkModeType>(proto_msg->work_mode_type());

  return 0;
}

}  // namespace autodrive