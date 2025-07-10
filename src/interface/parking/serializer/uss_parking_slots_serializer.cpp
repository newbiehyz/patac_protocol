//
#include "interface/parking/serializer/uss_parking_slots_serializer.h"

#include <utils/exception.h>

#include "interface/parking/uss_parking_slots_data.h"
#include "protocol/uss_parking_slots.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int UssParkingSlotsDataSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& uss_park_slots{data_msg->GetUssParkingSlots()};
  proto_msg->set_version(uss_park_slots.version);

  for (std::uint16_t h_idx = 0; h_idx < uss_park_slots.num_uss_header;
       h_idx++) {
    auto& uss_header_data{uss_park_slots.uss_header.at(h_idx)};
    workflow::proto::PerFusUssHeader* uss_header_proto =
        proto_msg->add_uss_header();
    uss_header_proto->set_mcu_send_stamp_si64(
        uss_header_data.mcu_send_stamp_si64);
    uss_header_proto->set_mcu_send_wave_stamp_si64(
        uss_header_data.mcu_send_wave_stamp_si64);
    uss_header_proto->set_mesa_group_tx_mask_ui16(
        uss_header_data.mesa_group_tx_mask_ui16);
    uss_header_proto->set_mesa_group_rx_mask_ui16(
        uss_header_data.mesa_group_rx_mask_ui16);

    uss_header_proto->set_uss_header_group_id(
        static_cast<workflow::proto::UssHeaderGroupIdEn>(
            uss_header_data.uss_header_group_id));
    uss_header_proto->set_uss_header_freq_mode(
        static_cast<workflow::proto::FreqModeEn>(
            uss_header_data.uss_header_freq_mode));
  }  // end uss_header

  for (std::uint16_t h_idx = 0; h_idx < uss_park_slots.num_uss_ps; h_idx++) {
    auto& uss_ps_data{uss_park_slots.uss_ps.at(h_idx)};
    workflow::proto::PerFusUssPs* uss_ps_proto = proto_msg->add_uss_ps();
    uss_ps_proto->set_uss_ps_id_ui8(uss_ps_data.uss_ps_id_ui8);
    uss_ps_proto->set_uss_ps_status_en(
        static_cast<workflow::proto::UssPsStatusEn>(
            uss_ps_data.uss_ps_status_en));
    uss_ps_proto->set_uss_ps_type_en(
        static_cast<workflow::proto::UssPsTypeEn>(uss_ps_data.uss_ps_type_en));
    uss_ps_proto->set_uss_ps_orientation_en(
        static_cast<workflow::proto::UssPsOrientationEn>(
            uss_ps_data.uss_ps_orientation_en));

    uss_ps_proto->set_uss_ps_1st_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            uss_ps_data.uss_ps_1st_boundary_type_en));
    uss_ps_proto->set_uss_ps_2nd_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            uss_ps_data.uss_ps_2nd_boundary_type_en));
    uss_ps_proto->set_uss_ps_cross_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            uss_ps_data.uss_ps_cross_boundary_type_en));

    uss_ps_proto->set_uss_ps_depth_ui16(uss_ps_data.uss_ps_depth_ui16);
    uss_ps_proto->set_uss_ps_length_ui16(uss_ps_data.uss_ps_length_ui16);
    uss_ps_proto->set_uss_ps_angle_i16(uss_ps_data.uss_ps_angle_i16);

    uss_ps_proto->mutable_uss_ps_1st_corner()->set_x_si32(
        uss_ps_data.uss_ps_1st_corner.x_si32);
    uss_ps_proto->mutable_uss_ps_1st_corner()->set_y_si32(
        uss_ps_data.uss_ps_1st_corner.y_si32);

    uss_ps_proto->mutable_uss_ps_2nd_corner()->set_x_si32(
        uss_ps_data.uss_ps_2nd_corner.x_si32);
    uss_ps_proto->mutable_uss_ps_2nd_corner()->set_y_si32(
        uss_ps_data.uss_ps_2nd_corner.y_si32);

    uss_ps_proto->mutable_uss_ps_3st_corner()->set_x_si32(
        uss_ps_data.uss_ps_3st_corner.x_si32);
    uss_ps_proto->mutable_uss_ps_3st_corner()->set_y_si32(
        uss_ps_data.uss_ps_3st_corner.y_si32);

    uss_ps_proto->mutable_uss_ps_4nd_corner()->set_x_si32(
        uss_ps_data.uss_ps_4nd_corner.x_si32);
    uss_ps_proto->mutable_uss_ps_4nd_corner()->set_y_si32(
        uss_ps_data.uss_ps_4nd_corner.y_si32);

    uss_ps_proto->mutable_uss_ps_1st_corner_gcs()->set_x_si32(
        uss_ps_data.uss_ps_1st_corner_gcs.x_si32);
    uss_ps_proto->mutable_uss_ps_1st_corner_gcs()->set_y_si32(
        uss_ps_data.uss_ps_1st_corner_gcs.y_si32);

    uss_ps_proto->mutable_uss_ps_2nd_corner_gcs()->set_x_si32(
        uss_ps_data.uss_ps_2nd_corner_gcs.x_si32);
    uss_ps_proto->mutable_uss_ps_2nd_corner_gcs()->set_y_si32(
        uss_ps_data.uss_ps_2nd_corner_gcs.y_si32);

    uss_ps_proto->mutable_uss_ps_3st_corner_gcs()->set_x_si32(
        uss_ps_data.uss_ps_3st_corner_gcs.x_si32);
    uss_ps_proto->mutable_uss_ps_3st_corner_gcs()->set_y_si32(
        uss_ps_data.uss_ps_3st_corner_gcs.y_si32);

    uss_ps_proto->mutable_uss_ps_4nd_corner_gcs()->set_x_si32(
        uss_ps_data.uss_ps_4nd_corner_gcs.x_si32);
    uss_ps_proto->mutable_uss_ps_4nd_corner_gcs()->set_y_si32(
        uss_ps_data.uss_ps_4nd_corner_gcs.y_si32);

    uss_ps_proto->mutable_uss_ps_boundary_1st_info()->set_x_si32(
        uss_ps_data.uss_ps_boundary_1st_info.X_si16);
    uss_ps_proto->mutable_uss_ps_boundary_1st_info()->set_y_si32(
        uss_ps_data.uss_ps_boundary_1st_info.Y_si16);

    uss_ps_proto->mutable_uss_ps_boundary_2nd_info()->set_x_si32(
        uss_ps_data.uss_ps_boundary_2nd_info.X_si16);
    uss_ps_proto->mutable_uss_ps_boundary_2nd_info()->set_y_si32(
        uss_ps_data.uss_ps_boundary_2nd_info.Y_si16);

    uss_ps_proto->set_uss_ps_finish_timestamp(
        uss_ps_data.uss_ps_finish_timestamp);
  }  // end uss_ps

  {
    auto& uss_correct_data{uss_park_slots.uss_correct_ps};
    workflow::proto::PerFusUssPs* uss_correct_proto =
        proto_msg->mutable_uss_correct_ps();
    uss_correct_proto->set_uss_ps_id_ui8(uss_correct_data.uss_ps_id_ui8);
    uss_correct_proto->set_uss_ps_status_en(
        static_cast<workflow::proto::UssPsStatusEn>(
            uss_correct_data.uss_ps_status_en));
    uss_correct_proto->set_uss_ps_type_en(
        static_cast<workflow::proto::UssPsTypeEn>(
            uss_correct_data.uss_ps_type_en));
    uss_correct_proto->set_uss_ps_orientation_en(
        static_cast<workflow::proto::UssPsOrientationEn>(
            uss_correct_data.uss_ps_orientation_en));

    uss_correct_proto->set_uss_ps_1st_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            uss_correct_data.uss_ps_1st_boundary_type_en));
    uss_correct_proto->set_uss_ps_2nd_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            uss_correct_data.uss_ps_2nd_boundary_type_en));
    uss_correct_proto->set_uss_ps_cross_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            uss_correct_data.uss_ps_cross_boundary_type_en));

    uss_correct_proto->set_uss_ps_depth_ui16(
        uss_correct_data.uss_ps_depth_ui16);
    uss_correct_proto->set_uss_ps_length_ui16(
        uss_correct_data.uss_ps_length_ui16);
    uss_correct_proto->set_uss_ps_angle_i16(
        uss_correct_data.uss_ps_angle_i16);

    uss_correct_proto->mutable_uss_ps_1st_corner()->set_x_si32(
        uss_correct_data.uss_ps_1st_corner.x_si32);
    uss_correct_proto->mutable_uss_ps_1st_corner()->set_y_si32(
        uss_correct_data.uss_ps_1st_corner.y_si32);

    uss_correct_proto->mutable_uss_ps_2nd_corner()->set_x_si32(
        uss_correct_data.uss_ps_2nd_corner.x_si32);
    uss_correct_proto->mutable_uss_ps_2nd_corner()->set_y_si32(
        uss_correct_data.uss_ps_2nd_corner.y_si32);

    uss_correct_proto->mutable_uss_ps_3st_corner()->set_x_si32(
        uss_correct_data.uss_ps_3st_corner.x_si32);
    uss_correct_proto->mutable_uss_ps_3st_corner()->set_y_si32(
        uss_correct_data.uss_ps_3st_corner.y_si32);

    uss_correct_proto->mutable_uss_ps_4nd_corner()->set_x_si32(
        uss_correct_data.uss_ps_4nd_corner.x_si32);
    uss_correct_proto->mutable_uss_ps_4nd_corner()->set_y_si32(
        uss_correct_data.uss_ps_4nd_corner.y_si32);

    uss_correct_proto->mutable_uss_ps_1st_corner_gcs()->set_x_si32(
        uss_correct_data.uss_ps_1st_corner_gcs.x_si32);
    uss_correct_proto->mutable_uss_ps_1st_corner_gcs()->set_y_si32(
        uss_correct_data.uss_ps_1st_corner_gcs.y_si32);

    uss_correct_proto->mutable_uss_ps_2nd_corner_gcs()->set_x_si32(
        uss_correct_data.uss_ps_2nd_corner_gcs.x_si32);
    uss_correct_proto->mutable_uss_ps_2nd_corner_gcs()->set_y_si32(
        uss_correct_data.uss_ps_2nd_corner_gcs.y_si32);

    uss_correct_proto->mutable_uss_ps_3st_corner_gcs()->set_x_si32(
        uss_correct_data.uss_ps_3st_corner_gcs.x_si32);
    uss_correct_proto->mutable_uss_ps_3st_corner_gcs()->set_y_si32(
        uss_correct_data.uss_ps_3st_corner_gcs.y_si32);

    uss_correct_proto->mutable_uss_ps_4nd_corner_gcs()->set_x_si32(
        uss_correct_data.uss_ps_4nd_corner_gcs.x_si32);
    uss_correct_proto->mutable_uss_ps_4nd_corner_gcs()->set_y_si32(
        uss_correct_data.uss_ps_4nd_corner_gcs.y_si32);

    uss_correct_proto->mutable_uss_ps_boundary_1st_info()->set_x_si32(
        uss_correct_data.uss_ps_boundary_1st_info.X_si16);
    uss_correct_proto->mutable_uss_ps_boundary_1st_info()->set_y_si32(
        uss_correct_data.uss_ps_boundary_1st_info.Y_si16);

    uss_correct_proto->mutable_uss_ps_boundary_2nd_info()->set_x_si32(
        uss_correct_data.uss_ps_boundary_2nd_info.X_si16);
    uss_correct_proto->mutable_uss_ps_boundary_2nd_info()->set_y_si32(
        uss_correct_data.uss_ps_boundary_2nd_info.Y_si16);

    uss_correct_proto->set_uss_ps_finish_timestamp(
        uss_correct_data.uss_ps_finish_timestamp);
  }  // end uss_correct_ps

  {
    auto& machine_data{uss_park_slots.state_machine};
    workflow::proto::StateMachine* machine_proto =
        proto_msg->mutable_state_machine();
    machine_proto->set_plv_state(
        static_cast<workflow::proto::PlvState>(machine_data.plv_state));
    machine_proto->set_psu_state(
        static_cast<workflow::proto::PsuState>(machine_data.psu_state));
    machine_proto->set_odo_state(
        static_cast<workflow::proto::OdoState>(machine_data.odo_state));
  }  // end state_machine

  proto_msg->set_sensor_stamp(uss_park_slots.sensor_stamp);
  proto_msg->set_sample_stamp(uss_park_slots.sample_stamp);

  return 0;
}

int UssParkingSlotsDataSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& uss_park_slots_data{data_msg->MutableUssParkingSlots()};

  uss_park_slots_data.version = proto_msg->version();

  uss_park_slots_data.num_uss_header = proto_msg->uss_header_size();
  for (std::uint16_t h_idx = 0; h_idx < uss_park_slots_data.num_uss_header;
       h_idx++) {
    auto& uss_header_data{uss_park_slots_data.uss_header.at(h_idx)};
    auto& uss_header_proto{proto_msg->uss_header(h_idx)};
    uss_header_data.mcu_send_stamp_si64 =
        uss_header_proto.mcu_send_stamp_si64();
    uss_header_data.mcu_send_wave_stamp_si64 =
        uss_header_proto.mcu_send_wave_stamp_si64();

    uss_header_data.mesa_group_tx_mask_ui16 =
        static_cast<std::uint16_t>(uss_header_proto.mesa_group_tx_mask_ui16());
    uss_header_data.mesa_group_rx_mask_ui16 =
        static_cast<std::uint16_t>(uss_header_proto.mesa_group_rx_mask_ui16());

    uss_header_data.uss_header_group_id =
        static_cast<UssHeaderGroupIdEn>(uss_header_proto.uss_header_group_id());
    uss_header_data.uss_header_freq_mode =
        static_cast<FreqModeEn>(uss_header_proto.uss_header_freq_mode());
  }  // end uss_header

  uss_park_slots_data.num_uss_ps = proto_msg->uss_ps_size();
  for (std::uint16_t p_idx = 0; p_idx < uss_park_slots_data.num_uss_ps;
       p_idx++) {
    auto& uss_ps_data{uss_park_slots_data.uss_ps.at(p_idx)};
    auto& uss_ps_proto{proto_msg->uss_ps(p_idx)};
    uss_ps_data.uss_ps_id_ui8 =
        static_cast<std::uint8_t>(uss_ps_proto.uss_ps_id_ui8());
    uss_ps_data.uss_ps_status_en =
        static_cast<UssParkingSlotsData::UssPsStatusEn>(
            uss_ps_proto.uss_ps_status_en());
    uss_ps_data.uss_ps_type_en = static_cast<UssParkingSlotsData::UssPsTypeEn>(
        uss_ps_proto.uss_ps_type_en());
    uss_ps_data.uss_ps_orientation_en =
        static_cast<UssParkingSlotsData::UssPsOrientationEn>(
            uss_ps_proto.uss_ps_orientation_en());

    uss_ps_data.uss_ps_1st_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            uss_ps_proto.uss_ps_1st_boundary_type_en());
    uss_ps_data.uss_ps_2nd_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            uss_ps_proto.uss_ps_2nd_boundary_type_en());
    uss_ps_data.uss_ps_cross_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            uss_ps_proto.uss_ps_cross_boundary_type_en());

    uss_ps_data.uss_ps_depth_ui16 =
        static_cast<std::uint16_t>(uss_ps_proto.uss_ps_depth_ui16());
    uss_ps_data.uss_ps_length_ui16 =
        static_cast<std::uint16_t>(uss_ps_proto.uss_ps_length_ui16());
    uss_ps_data.uss_ps_angle_i16 =
        static_cast<std::int16_t>(uss_ps_proto.uss_ps_angle_i16());

    if (uss_ps_proto.has_uss_ps_1st_corner()) {
      uss_ps_data.uss_ps_1st_corner.x_si32 =
          uss_ps_proto.uss_ps_1st_corner().x_si32();
      uss_ps_data.uss_ps_1st_corner.y_si32 =
          uss_ps_proto.uss_ps_1st_corner().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_2nd_corner()) {
      uss_ps_data.uss_ps_2nd_corner.x_si32 =
          uss_ps_proto.uss_ps_2nd_corner().x_si32();
      uss_ps_data.uss_ps_2nd_corner.y_si32 =
          uss_ps_proto.uss_ps_2nd_corner().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_3st_corner()) {
      uss_ps_data.uss_ps_3st_corner.x_si32 =
          uss_ps_proto.uss_ps_3st_corner().x_si32();
      uss_ps_data.uss_ps_3st_corner.y_si32 =
          uss_ps_proto.uss_ps_3st_corner().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_4nd_corner()) {
      uss_ps_data.uss_ps_4nd_corner.x_si32 =
          uss_ps_proto.uss_ps_4nd_corner().x_si32();
      uss_ps_data.uss_ps_4nd_corner.y_si32 =
          uss_ps_proto.uss_ps_4nd_corner().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_1st_corner_gcs()) {
      uss_ps_data.uss_ps_1st_corner_gcs.x_si32 =
          uss_ps_proto.uss_ps_1st_corner_gcs().x_si32();
      uss_ps_data.uss_ps_1st_corner_gcs.y_si32 =
          uss_ps_proto.uss_ps_1st_corner_gcs().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_2nd_corner_gcs()) {
      uss_ps_data.uss_ps_2nd_corner_gcs.x_si32 =
          uss_ps_proto.uss_ps_2nd_corner_gcs().x_si32();
      uss_ps_data.uss_ps_2nd_corner_gcs.y_si32 =
          uss_ps_proto.uss_ps_2nd_corner_gcs().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_3st_corner_gcs()) {
      uss_ps_data.uss_ps_3st_corner_gcs.x_si32 =
          uss_ps_proto.uss_ps_3st_corner_gcs().x_si32();
      uss_ps_data.uss_ps_3st_corner_gcs.y_si32 =
          uss_ps_proto.uss_ps_3st_corner_gcs().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_4nd_corner_gcs()) {
      uss_ps_data.uss_ps_4nd_corner_gcs.x_si32 =
          uss_ps_proto.uss_ps_4nd_corner_gcs().x_si32();
      uss_ps_data.uss_ps_4nd_corner_gcs.y_si32 =
          uss_ps_proto.uss_ps_4nd_corner_gcs().y_si32();
    }

    if (uss_ps_proto.has_uss_ps_boundary_1st_info()) {
      uss_ps_data.uss_ps_boundary_1st_info.X_si16 = static_cast<std::uint16_t>(
          uss_ps_proto.uss_ps_boundary_1st_info().x_si32());
      uss_ps_data.uss_ps_boundary_1st_info.Y_si16 = static_cast<std::uint16_t>(
          uss_ps_proto.uss_ps_boundary_1st_info().y_si32());
    }

    if (uss_ps_proto.has_uss_ps_boundary_2nd_info()) {
      uss_ps_data.uss_ps_boundary_2nd_info.X_si16 = static_cast<std::uint16_t>(
          uss_ps_proto.uss_ps_boundary_2nd_info().x_si32());
      uss_ps_data.uss_ps_boundary_2nd_info.Y_si16 = static_cast<std::uint16_t>(
          uss_ps_proto.uss_ps_boundary_2nd_info().y_si32());
    }

    uss_ps_data.uss_ps_finish_timestamp =
        uss_ps_proto.uss_ps_finish_timestamp();
  }  // end uss_ps

  {
    auto& uss_correct_ps_data{uss_park_slots_data.uss_correct_ps};
    auto& uss_correct_ps_proto{proto_msg->uss_correct_ps()};

    uss_correct_ps_data.uss_ps_id_ui8 =
        static_cast<std::uint8_t>(uss_correct_ps_proto.uss_ps_id_ui8());
    uss_correct_ps_data.uss_ps_status_en =
        static_cast<UssParkingSlotsData::UssPsStatusEn>(
            uss_correct_ps_proto.uss_ps_status_en());
    uss_correct_ps_data.uss_ps_type_en =
        static_cast<UssParkingSlotsData::UssPsTypeEn>(
            uss_correct_ps_proto.uss_ps_type_en());
    uss_correct_ps_data.uss_ps_orientation_en =
        static_cast<UssParkingSlotsData::UssPsOrientationEn>(
            uss_correct_ps_proto.uss_ps_orientation_en());

    uss_correct_ps_data.uss_ps_1st_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            uss_correct_ps_proto.uss_ps_1st_boundary_type_en());
    uss_correct_ps_data.uss_ps_2nd_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            uss_correct_ps_proto.uss_ps_2nd_boundary_type_en());
    uss_correct_ps_data.uss_ps_cross_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            uss_correct_ps_proto.uss_ps_cross_boundary_type_en());

    uss_correct_ps_data.uss_ps_depth_ui16 =
        static_cast<std::uint16_t>(uss_correct_ps_proto.uss_ps_depth_ui16());
    uss_correct_ps_data.uss_ps_length_ui16 =
        static_cast<std::uint16_t>(uss_correct_ps_proto.uss_ps_length_ui16());
    uss_correct_ps_data.uss_ps_angle_i16 =
        static_cast<std::int16_t>(uss_correct_ps_proto.uss_ps_angle_i16());

    if (uss_correct_ps_proto.has_uss_ps_1st_corner()) {
      uss_correct_ps_data.uss_ps_1st_corner.x_si32 =
          uss_correct_ps_proto.uss_ps_1st_corner().x_si32();
      uss_correct_ps_data.uss_ps_1st_corner.y_si32 =
          uss_correct_ps_proto.uss_ps_1st_corner().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_2nd_corner()) {
      uss_correct_ps_data.uss_ps_2nd_corner.x_si32 =
          uss_correct_ps_proto.uss_ps_2nd_corner().x_si32();
      uss_correct_ps_data.uss_ps_2nd_corner.y_si32 =
          uss_correct_ps_proto.uss_ps_2nd_corner().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_3st_corner()) {
      uss_correct_ps_data.uss_ps_3st_corner.x_si32 =
          uss_correct_ps_proto.uss_ps_3st_corner().x_si32();
      uss_correct_ps_data.uss_ps_3st_corner.y_si32 =
          uss_correct_ps_proto.uss_ps_3st_corner().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_4nd_corner()) {
      uss_correct_ps_data.uss_ps_4nd_corner.x_si32 =
          uss_correct_ps_proto.uss_ps_4nd_corner().x_si32();
      uss_correct_ps_data.uss_ps_4nd_corner.y_si32 =
          uss_correct_ps_proto.uss_ps_4nd_corner().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_1st_corner_gcs()) {
      uss_correct_ps_data.uss_ps_1st_corner_gcs.x_si32 =
          uss_correct_ps_proto.uss_ps_1st_corner_gcs().x_si32();
      uss_correct_ps_data.uss_ps_1st_corner_gcs.y_si32 =
          uss_correct_ps_proto.uss_ps_1st_corner_gcs().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_2nd_corner_gcs()) {
      uss_correct_ps_data.uss_ps_2nd_corner_gcs.x_si32 =
          uss_correct_ps_proto.uss_ps_2nd_corner_gcs().x_si32();
      uss_correct_ps_data.uss_ps_2nd_corner_gcs.y_si32 =
          uss_correct_ps_proto.uss_ps_2nd_corner_gcs().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_3st_corner_gcs()) {
      uss_correct_ps_data.uss_ps_3st_corner_gcs.x_si32 =
          uss_correct_ps_proto.uss_ps_3st_corner_gcs().x_si32();
      uss_correct_ps_data.uss_ps_3st_corner_gcs.y_si32 =
          uss_correct_ps_proto.uss_ps_3st_corner_gcs().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_4nd_corner_gcs()) {
      uss_correct_ps_data.uss_ps_4nd_corner_gcs.x_si32 =
          uss_correct_ps_proto.uss_ps_4nd_corner_gcs().x_si32();
      uss_correct_ps_data.uss_ps_4nd_corner_gcs.y_si32 =
          uss_correct_ps_proto.uss_ps_4nd_corner_gcs().y_si32();
    }

    if (uss_correct_ps_proto.has_uss_ps_boundary_1st_info()) {
      uss_correct_ps_data.uss_ps_boundary_1st_info.X_si16 =
          static_cast<std::uint16_t>(
              uss_correct_ps_proto.uss_ps_boundary_1st_info().x_si32());
      uss_correct_ps_data.uss_ps_boundary_1st_info.Y_si16 =
          static_cast<std::uint16_t>(
              uss_correct_ps_proto.uss_ps_boundary_1st_info().y_si32());
    }

    if (uss_correct_ps_proto.has_uss_ps_boundary_2nd_info()) {
      uss_correct_ps_data.uss_ps_boundary_2nd_info.X_si16 =
          static_cast<std::uint16_t>(
              uss_correct_ps_proto.uss_ps_boundary_2nd_info().x_si32());
      uss_correct_ps_data.uss_ps_boundary_2nd_info.Y_si16 =
          static_cast<std::uint16_t>(
              uss_correct_ps_proto.uss_ps_boundary_2nd_info().y_si32());
    }

    uss_correct_ps_data.uss_ps_finish_timestamp =
        uss_correct_ps_proto.uss_ps_finish_timestamp();
  }  // end uss_correct_ps

  {
    auto& machien_data{uss_park_slots_data.state_machine};
    auto& machine_proto{proto_msg->state_machine()};
    machien_data.plv_state = static_cast<PlvState>(machine_proto.plv_state());
    machien_data.psu_state = static_cast<PsuState>(machine_proto.psu_state());
    machien_data.odo_state = static_cast<OdoState>(machine_proto.odo_state());
  }  // end state_machine

  uss_park_slots_data.sensor_stamp = proto_msg->sensor_stamp();
  uss_park_slots_data.sample_stamp = proto_msg->sample_stamp();


  return 0;
}

}  // namespace autodrive