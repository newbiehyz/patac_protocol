//
#include "interface/parking/serializer/uss_obstacles_serializer.h"

#include <utils/exception.h>

#include "interface/parking/uss_obstacles_data.h"
#include "protocol/uss_obstacles.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int UssObstaclesDataSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& uss_obs_data{data_msg->GetUssObstaclesData()};
  proto_msg->set_version(uss_obs_data.version);

  for (std::uint8_t uh_idx = 0; uh_idx < uss_obs_data.num_uss_header;
       uh_idx++) {
    auto& uss_header_data{uss_obs_data.uss_header.at(uh_idx)};
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


  for (std::uint8_t ur_idx = 0; ur_idx < uss_obs_data.num_uss_raw; ur_idx++) {
    auto& uss_raw_data{uss_obs_data.uss_raw.at(ur_idx)};
    workflow::proto::PerFusUssRawData* uss_raw_proto = proto_msg->add_uss_raw();
    uss_raw_proto->set_bind_zone_flag(uss_raw_data.bind_zone_flag);
    uss_raw_proto->set_ring_time(uss_raw_data.ring_time);
    uss_raw_proto->set_bind_zone_dis(uss_raw_data.bind_zone_dis);
    uss_raw_proto->set_time_diff_tx_rx(uss_raw_data.time_diff_tx_rx);
    uss_raw_proto->set_env_noise(uss_raw_data.env_noise);
    for (std::uint8_t e_idx = 0; e_idx < uss_raw_data.num_echo; e_idx++) {
      uss_raw_proto->add_echo_cfd(uss_raw_data.echo_cfd.at(e_idx));
      uss_raw_proto->add_echo_dist(uss_raw_data.echo_dist.at(e_idx));
      uss_raw_proto->add_echo_cfd(uss_raw_data.echo_cfd.at(e_idx));
      uss_raw_proto->add_echo_high(uss_raw_data.echo_high.at(e_idx));
      uss_raw_proto->add_echo_width(uss_raw_data.echo_width.at(e_idx));
      uss_raw_proto->add_echo_mt(uss_raw_data.echo_mt.at(e_idx));
    }
    uss_raw_proto->set_echo_send_timestamp(uss_raw_data.echo_send_timestamp);
    uss_raw_proto->set_per_fus_sns_id(
        static_cast<workflow::proto::PerFusSnsID>(uss_raw_data.per_fus_sns_id));
    uss_raw_proto->set_per_fus_echo_mode(
        static_cast<workflow::proto::PerFusEchoMode>(
            uss_raw_data.per_fus_echo_mode));
    uss_raw_proto->set_per_fus_echo_source_id(
        static_cast<workflow::proto::PerFusSnsID>(
            uss_raw_data.per_fus_echo_source_id));
  }  // end uss_raw
  auto &uss_ce_raw_obj{uss_obs_data.uss_ce_raw_obj};
  workflow::proto::PerFusUssAllCERawData *uss_ce_raw_obj_proto =
      proto_msg->mutable_uss_ce_raw_obj();
  uss_ce_raw_obj_proto->set_ce1_to_2distance(uss_ce_raw_obj.ce1_to_2distance);
  uss_ce_raw_obj_proto->set_ce2_to_1distance(uss_ce_raw_obj.ce2_to_1distance);
  uss_ce_raw_obj_proto->set_ce2_to_3distance(uss_ce_raw_obj.ce2_to_3distance);
  uss_ce_raw_obj_proto->set_ce3_to_2distance(uss_ce_raw_obj.ce3_to_2distance);
  uss_ce_raw_obj_proto->set_ce3_to_4distance(uss_ce_raw_obj.ce3_to_4distance);
  uss_ce_raw_obj_proto->set_ce4_to_3distance(uss_ce_raw_obj.ce4_to_3distance);
  uss_ce_raw_obj_proto->set_ce4_to_5distance(uss_ce_raw_obj.ce4_to_5distance);
  uss_ce_raw_obj_proto->set_ce5_to_4distance(uss_ce_raw_obj.ce5_to_4distance);
  uss_ce_raw_obj_proto->set_ce5_to_6distance(uss_ce_raw_obj.ce5_to_6distance);
  uss_ce_raw_obj_proto->set_ce6_to_5distance(uss_ce_raw_obj.ce6_to_5distance);
  uss_ce_raw_obj_proto->set_ce7_to_8distance(uss_ce_raw_obj.ce7_to_8distance);
  uss_ce_raw_obj_proto->set_ce8_to_7distance(uss_ce_raw_obj.ce8_to_7distance);
  uss_ce_raw_obj_proto->set_ce8_to_9distance(uss_ce_raw_obj.ce8_to_9distance);
  uss_ce_raw_obj_proto->set_ce9_to_8distance(uss_ce_raw_obj.ce9_to_8distance);
  uss_ce_raw_obj_proto->set_ce9_to_10distance(uss_ce_raw_obj.ce9_to_10distance);
  uss_ce_raw_obj_proto->set_ce10_to_9distance(uss_ce_raw_obj.ce10_to_9distance);
  uss_ce_raw_obj_proto->set_ce10_to_11distance(uss_ce_raw_obj.ce10_to_11distance);
  uss_ce_raw_obj_proto->set_ce11_to_10distance(uss_ce_raw_obj.ce11_to_10distance);
  uss_ce_raw_obj_proto->set_ce11_to_12distance(uss_ce_raw_obj.ce11_to_12distance);
  uss_ce_raw_obj_proto->set_ce12_to_11distance(uss_ce_raw_obj.ce12_to_11distance);
  uss_ce_raw_obj_proto->set_ce1_to_2timestamp(uss_ce_raw_obj.ce1_to_2timestamp);
  uss_ce_raw_obj_proto->set_ce2_to_1timestamp(uss_ce_raw_obj.ce2_to_1timestamp);
  uss_ce_raw_obj_proto->set_ce2_to_3timestamp(uss_ce_raw_obj.ce2_to_3timestamp);
  uss_ce_raw_obj_proto->set_ce3_to_2timestamp(uss_ce_raw_obj.ce3_to_2timestamp);
  uss_ce_raw_obj_proto->set_ce3_to_4timestamp(uss_ce_raw_obj.ce3_to_4timestamp);
  uss_ce_raw_obj_proto->set_ce4_to_3timestamp(uss_ce_raw_obj.ce4_to_3timestamp);
  uss_ce_raw_obj_proto->set_ce4_to_5timestamp(uss_ce_raw_obj.ce4_to_5timestamp);
  uss_ce_raw_obj_proto->set_ce5_to_4timestamp(uss_ce_raw_obj.ce5_to_4timestamp);
  uss_ce_raw_obj_proto->set_ce5_to_6timestamp(uss_ce_raw_obj.ce5_to_6timestamp);
  uss_ce_raw_obj_proto->set_ce6_to_5timestamp(uss_ce_raw_obj.ce6_to_5timestamp);
  uss_ce_raw_obj_proto->set_ce7_to_8timestamp(uss_ce_raw_obj.ce7_to_8timestamp);
  uss_ce_raw_obj_proto->set_ce8_to_7timestamp(uss_ce_raw_obj.ce8_to_7timestamp);
  uss_ce_raw_obj_proto->set_ce8_to_9timestamp(uss_ce_raw_obj.ce8_to_9timestamp);
  uss_ce_raw_obj_proto->set_ce9_to_8timestamp(uss_ce_raw_obj.ce9_to_8timestamp);
  uss_ce_raw_obj_proto->set_ce9_to_10timestamp(uss_ce_raw_obj.ce9_to_10timestamp);
  uss_ce_raw_obj_proto->set_ce10_to_9timestamp(uss_ce_raw_obj.ce10_to_9timestamp);
  uss_ce_raw_obj_proto->set_ce10_to_11timestamp(uss_ce_raw_obj.ce10_to_11timestamp);
  uss_ce_raw_obj_proto->set_ce11_to_10timestamp(uss_ce_raw_obj.ce11_to_10timestamp);
  uss_ce_raw_obj_proto->set_ce11_to_12timestamp(uss_ce_raw_obj.ce11_to_12timestamp);
  uss_ce_raw_obj_proto->set_ce12_to_11timestamp(uss_ce_raw_obj.ce12_to_11timestamp);
  // end uss_ce_raw_obj

  for (std::uint8_t o_idx = 0; o_idx < uss_obs_data.num_uss_obj; o_idx++) {
    auto& uss_obj_data{uss_obs_data.uss_obj.at(o_idx)};
    workflow::proto::PerFusUssObj* uss_obj_proto = proto_msg->add_uss_obj();
    uss_obj_proto->set_uss_obj_x_id_ui8(uss_obj_data.uss_obj_x_id_ui8);
    uss_obj_proto->set_uss_obj_x_cfd_ui8(uss_obj_data.uss_obj_x_cfd_ui8);
    uss_obj_proto->set_uss_obj_x_type_en(
        static_cast<workflow::proto::UssObjXTypeEn>(
            uss_obj_data.uss_obj_x_type_en));
    uss_obj_proto->set_uss_obj_x_height_en(
        static_cast<workflow::proto::UssObjXHeightEn>(
            uss_obj_data.uss_obj_x_height_en));
    uss_obj_proto->set_uss_obj_x_on_course_en(
        static_cast<workflow::proto::UssObjXOnCourseEn>(
            uss_obj_data.uss_obj_x_on_course_en));
    uss_obj_proto->set_finish_timestamp_si64(
        uss_obj_data.finish_timestamp_si64);

    uss_obj_proto->mutable_uss_obj_x_pos_1()->set_x_si32(
        uss_obj_data.uss_obj_x_pos_1.X_si16);
    uss_obj_proto->mutable_uss_obj_x_pos_1()->set_y_si32(
        uss_obj_data.uss_obj_x_pos_1.Y_si16);

    uss_obj_proto->mutable_uss_obj_x_pos_2()->set_x_si32(
        uss_obj_data.uss_obj_x_pos_2.X_si16);
    uss_obj_proto->mutable_uss_obj_x_pos_2()->set_y_si32(
        uss_obj_data.uss_obj_x_pos_2.Y_si16);

    uss_obj_proto->mutable_uss_obj_x_err_pos_1()->set_x_si32(
        uss_obj_data.uss_obj_x_err_pos_1.X_si16);
    uss_obj_proto->mutable_uss_obj_x_err_pos_1()->set_y_si32(
        uss_obj_data.uss_obj_x_err_pos_1.Y_si16);

    uss_obj_proto->mutable_uss_obj_x_err_pos_2()->set_x_si32(
        uss_obj_data.uss_obj_x_err_pos_2.X_si16);
    uss_obj_proto->mutable_uss_obj_x_err_pos_2()->set_y_si32(
        uss_obj_data.uss_obj_x_err_pos_2.Y_si16);
    uss_obj_proto->set_obj_history_ui8(uss_obj_data.objHistory_ui8);
  }    // end uss_obj

  {
    auto& machine_data{uss_obs_data.state_machine};
    workflow::proto::StateMachine* machine_proto =
        proto_msg->mutable_state_machine();
    machine_proto->set_plv_state(
        static_cast<workflow::proto::PlvState>(machine_data.plv_state));
    machine_proto->set_psu_state(
        static_cast<workflow::proto::PsuState>(machine_data.psu_state));
    machine_proto->set_odo_state(
        static_cast<workflow::proto::OdoState>(machine_data.odo_state));
  }  // end state_machine

  proto_msg->set_sensor_stamp(uss_obs_data.sensor_stamp);
  proto_msg->set_sample_stamp(uss_obs_data.sample_stamp);

  return 0;
}

int UssObstaclesDataSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& uss_obs_data{data_msg->MutableUssObstaclesData()};

  uss_obs_data.version = proto_msg->version();

  uss_obs_data.num_uss_header = proto_msg->uss_header_size();
  for (std::uint8_t h_idx = 0; h_idx < uss_obs_data.num_uss_header; h_idx++) {
    auto& uss_header_data{uss_obs_data.uss_header.at(h_idx)};
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

  uss_obs_data.num_uss_raw = proto_msg->uss_raw_size();
  for (std::uint8_t u_idx = 0; u_idx < uss_obs_data.num_uss_raw; u_idx++) {
    auto& uss_raw_data{uss_obs_data.uss_raw.at(u_idx)};
    auto& uss_raw_proto{proto_msg->uss_raw(u_idx)};
    uss_raw_data.bind_zone_flag =
        static_cast<std::uint8_t>(uss_raw_proto.bind_zone_flag());
    uss_raw_data.ring_time =
        static_cast<std::uint16_t>(uss_raw_proto.ring_time());
    uss_raw_data.bind_zone_dis =
        static_cast<std::uint16_t>(uss_raw_proto.bind_zone_dis());
    uss_raw_data.time_diff_tx_rx =
        static_cast<std::uint16_t>(uss_raw_proto.time_diff_tx_rx());
    uss_raw_data.env_noise =
        static_cast<std::uint16_t>(uss_raw_proto.env_noise());

    uss_raw_data.num_echo = uss_raw_proto.echo_dist_size();
    for (std::uint8_t e_idx = 0; e_idx < uss_raw_data.num_echo; e_idx++) {
      uss_raw_data.echo_dist.at(e_idx) = uss_raw_proto.echo_dist(e_idx);
      uss_raw_data.echo_cfd.at(e_idx) = uss_raw_proto.echo_cfd(e_idx);
      uss_raw_data.echo_high.at(e_idx) = uss_raw_proto.echo_high(e_idx);
      uss_raw_data.echo_width.at(e_idx) = uss_raw_proto.echo_width(e_idx);
      uss_raw_data.echo_mt.at(e_idx) = uss_raw_proto.echo_mt(e_idx);
    }

    uss_raw_data.echo_send_timestamp = uss_raw_proto.echo_send_timestamp();

    uss_raw_data.per_fus_sns_id = static_cast<UssObstaclesData::PerFusSnsID>(
        uss_raw_proto.per_fus_sns_id());
    uss_raw_data.per_fus_echo_mode =
        static_cast<UssObstaclesData::PerFusEchoMode>(
            uss_raw_proto.per_fus_echo_mode());
    uss_raw_data.per_fus_echo_source_id =
        static_cast<UssObstaclesData::PerFusSnsID>(
            uss_raw_proto.per_fus_echo_source_id());
  }  // end uss_raw
   if (proto_msg->has_uss_ce_raw_obj()) {
    auto& uss_ce_raw_obj{uss_obs_data.uss_ce_raw_obj};
    auto& uss_ce_raw_obj_proto{proto_msg->uss_ce_raw_obj()};
    uss_ce_raw_obj.ce1_to_2distance = uss_ce_raw_obj_proto.ce1_to_2distance();
    uss_ce_raw_obj.ce2_to_1distance = uss_ce_raw_obj_proto.ce1_to_2distance();
    uss_ce_raw_obj.ce2_to_3distance = uss_ce_raw_obj_proto.ce2_to_3distance();
    uss_ce_raw_obj.ce3_to_2distance = uss_ce_raw_obj_proto.ce3_to_2distance();
    uss_ce_raw_obj.ce3_to_4distance = uss_ce_raw_obj_proto.ce3_to_4distance();
    uss_ce_raw_obj.ce4_to_3distance = uss_ce_raw_obj_proto.ce4_to_3distance();
    uss_ce_raw_obj.ce4_to_5distance = uss_ce_raw_obj_proto.ce4_to_5distance();
    uss_ce_raw_obj.ce5_to_4distance = uss_ce_raw_obj_proto.ce5_to_4distance();
    uss_ce_raw_obj.ce5_to_6distance = uss_ce_raw_obj_proto.ce5_to_6distance();
    uss_ce_raw_obj.ce6_to_5distance = uss_ce_raw_obj_proto.ce6_to_5distance();
    uss_ce_raw_obj.ce7_to_8distance = uss_ce_raw_obj_proto.ce7_to_8distance();
    uss_ce_raw_obj.ce8_to_7distance = uss_ce_raw_obj_proto.ce8_to_7distance();
    uss_ce_raw_obj.ce8_to_9distance = uss_ce_raw_obj_proto.ce8_to_9distance();
    uss_ce_raw_obj.ce9_to_8distance = uss_ce_raw_obj_proto.ce9_to_8distance();
    uss_ce_raw_obj.ce9_to_10distance = uss_ce_raw_obj_proto.ce9_to_10distance();
    uss_ce_raw_obj.ce10_to_9distance = uss_ce_raw_obj_proto.ce10_to_9distance();
    uss_ce_raw_obj.ce10_to_11distance = uss_ce_raw_obj_proto.ce10_to_11distance();
    uss_ce_raw_obj.ce11_to_10distance = uss_ce_raw_obj_proto.ce11_to_10distance();
    uss_ce_raw_obj.ce11_to_12distance = uss_ce_raw_obj_proto.ce11_to_12distance();
    uss_ce_raw_obj.ce12_to_11distance = uss_ce_raw_obj_proto.ce12_to_11distance();
    uss_ce_raw_obj.ce1_to_2timestamp = uss_ce_raw_obj_proto.ce1_to_2timestamp();
    uss_ce_raw_obj.ce2_to_1timestamp = uss_ce_raw_obj_proto.ce2_to_1timestamp();
    uss_ce_raw_obj.ce2_to_3timestamp = uss_ce_raw_obj_proto.ce2_to_3timestamp();
    uss_ce_raw_obj.ce3_to_2timestamp = uss_ce_raw_obj_proto.ce3_to_2timestamp();
    uss_ce_raw_obj.ce3_to_4timestamp = uss_ce_raw_obj_proto.ce3_to_4timestamp();
    uss_ce_raw_obj.ce4_to_3timestamp = uss_ce_raw_obj_proto.ce4_to_3timestamp();
    uss_ce_raw_obj.ce4_to_5timestamp = uss_ce_raw_obj_proto.ce4_to_5timestamp();
    uss_ce_raw_obj.ce5_to_4timestamp = uss_ce_raw_obj_proto.ce5_to_4timestamp();
    uss_ce_raw_obj.ce5_to_6timestamp = uss_ce_raw_obj_proto.ce5_to_6timestamp();
    uss_ce_raw_obj.ce6_to_5timestamp = uss_ce_raw_obj_proto.ce6_to_5timestamp();
    uss_ce_raw_obj.ce7_to_8timestamp = uss_ce_raw_obj_proto.ce7_to_8timestamp();
    uss_ce_raw_obj.ce8_to_7timestamp = uss_ce_raw_obj_proto.ce8_to_7timestamp();
    uss_ce_raw_obj.ce8_to_9timestamp = uss_ce_raw_obj_proto.ce8_to_9timestamp();
    uss_ce_raw_obj.ce9_to_8timestamp = uss_ce_raw_obj_proto.ce9_to_8timestamp();
    uss_ce_raw_obj.ce9_to_10timestamp = uss_ce_raw_obj_proto.ce9_to_10timestamp();
    uss_ce_raw_obj.ce10_to_9timestamp = uss_ce_raw_obj_proto.ce10_to_9timestamp();
    uss_ce_raw_obj.ce10_to_11timestamp = uss_ce_raw_obj_proto.ce10_to_11timestamp();
    uss_ce_raw_obj.ce11_to_10timestamp = uss_ce_raw_obj_proto.ce11_to_10timestamp();
    uss_ce_raw_obj.ce11_to_12timestamp = uss_ce_raw_obj_proto.ce11_to_12timestamp();
    uss_ce_raw_obj.ce12_to_11timestamp = uss_ce_raw_obj_proto.ce12_to_11timestamp();
  }  // end uss_ce_raw_obj

  uss_obs_data.num_uss_obj = proto_msg->uss_obj_size();
  for (std::uint8_t o_idx = 0; o_idx < uss_obs_data.num_uss_obj; o_idx++) {
    auto& uss_obj_data{uss_obs_data.uss_obj.at(o_idx)};
    auto& uss_obj_proto{proto_msg->uss_obj(o_idx)};

    uss_obj_data.uss_obj_x_id_ui8 =
        static_cast<std::uint8_t>(uss_obj_proto.uss_obj_x_id_ui8());
    uss_obj_data.uss_obj_x_cfd_ui8 =
        static_cast<std::uint8_t>(uss_obj_proto.uss_obj_x_cfd_ui8());
    uss_obj_data.uss_obj_x_type_en =
        static_cast<UssObstaclesData::UssObjXTypeEn>(
            uss_obj_proto.uss_obj_x_type_en());

    uss_obj_data.uss_obj_x_height_en =
        static_cast<UssObstaclesData::UssObjXHeightEn>(
            uss_obj_proto.uss_obj_x_height_en());
    uss_obj_data.uss_obj_x_on_course_en =
        static_cast<UssObstaclesData::UssObjXOnCourseEn>(
            uss_obj_proto.uss_obj_x_on_course_en());

    uss_obj_data.finish_timestamp_si64 = uss_obj_proto.finish_timestamp_si64();

    if (uss_obj_proto.has_uss_obj_x_pos_1()) {
      uss_obj_data.uss_obj_x_pos_1.X_si16 =
          uss_obj_proto.uss_obj_x_pos_1().x_si32();
      uss_obj_data.uss_obj_x_pos_1.Y_si16 =
          uss_obj_proto.uss_obj_x_pos_1().y_si32();
    }

    if (uss_obj_proto.has_uss_obj_x_pos_2()) {
      uss_obj_data.uss_obj_x_pos_2.X_si16 =
          uss_obj_proto.uss_obj_x_pos_2().x_si32();
      uss_obj_data.uss_obj_x_pos_2.Y_si16 =
          uss_obj_proto.uss_obj_x_pos_2().y_si32();
    }

    if (uss_obj_proto.has_uss_obj_x_err_pos_1()) {
      uss_obj_data.uss_obj_x_err_pos_1.X_si16 =
          uss_obj_proto.uss_obj_x_err_pos_1().x_si32();
      uss_obj_data.uss_obj_x_err_pos_1.Y_si16 =
          uss_obj_proto.uss_obj_x_err_pos_1().y_si32();
    }

    if (uss_obj_proto.has_uss_obj_x_err_pos_2()) {
      uss_obj_data.uss_obj_x_err_pos_2.X_si16 =
          uss_obj_proto.uss_obj_x_err_pos_2().x_si32();
      uss_obj_data.uss_obj_x_err_pos_2.Y_si16 =
          uss_obj_proto.uss_obj_x_err_pos_2().y_si32();
    }
    uss_obj_data.objHistory_ui8 = uss_obj_proto.obj_history_ui8();
  }  // end uss_obj

  if (proto_msg->has_state_machine()) {
    auto& machine_data{uss_obs_data.state_machine};
    auto& machine_proto{proto_msg->state_machine()};
    machine_data.plv_state = static_cast<PlvState>(machine_proto.plv_state());
    machine_data.psu_state = static_cast<PsuState>(machine_proto.psu_state());
    machine_data.odo_state = static_cast<OdoState>(machine_proto.odo_state());
  }  // end state_machine

  uss_obs_data.sensor_stamp = proto_msg->sensor_stamp();
  uss_obs_data.sample_stamp = proto_msg->sample_stamp();

  return 0;
}

}  // namespace autodrive