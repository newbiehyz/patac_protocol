//
#include "interface/parking/serializer/uss_tracking_mode_serializer.h"

#include <utils/exception.h>

#include "interface/parking/uss_tracking_mode_data.h"
#include "protocol/uss_tracking_mode.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int UssTrackingModeDataSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& uss_tracking_mode_data{data_msg->GetUssTrackingMode()};

  proto_msg->set_plv_mode(
      static_cast<workflow::proto::PlvState>(uss_tracking_mode_data.plv_mode));
  proto_msg->set_psu_mode(
      static_cast<workflow::proto::PsuState>(uss_tracking_mode_data.psu_mode));
  proto_msg->set_uss_ps_clear_distance(uss_tracking_mode_data.uss_ps_clear_distance);
  proto_msg->set_odo_mode(
      static_cast<workflow::proto::OdoState>(uss_tracking_mode_data.odo_mode));

  {
    auto& ps_correct_data{uss_tracking_mode_data.ps_correct};
    workflow::proto::PerFusUssPs* ps_correct_proto =
        proto_msg->mutable_ps_correct();
    ps_correct_proto->set_uss_ps_id_ui8(ps_correct_data.uss_ps_id_ui8);
    ps_correct_proto->set_uss_ps_status_en(
        static_cast<workflow::proto::UssPsStatusEn>(
            ps_correct_data.uss_ps_status_en));
    ps_correct_proto->set_uss_ps_type_en(
        static_cast<workflow::proto::UssPsTypeEn>(
            ps_correct_data.uss_ps_type_en));
    ps_correct_proto->set_uss_ps_orientation_en(
        static_cast<workflow::proto::UssPsOrientationEn>(
            ps_correct_data.uss_ps_orientation_en));

    ps_correct_proto->set_uss_ps_1st_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            ps_correct_data.uss_ps_1st_boundary_type_en));
    ps_correct_proto->set_uss_ps_2nd_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            ps_correct_data.uss_ps_2nd_boundary_type_en));
    ps_correct_proto->set_uss_ps_cross_boundary_type_en(
        static_cast<workflow::proto::UssPsBoundaryTypeEn>(
            ps_correct_data.uss_ps_cross_boundary_type_en));

    ps_correct_proto->set_uss_ps_depth_ui16(
        ps_correct_data.uss_ps_depth_ui16);
    ps_correct_proto->set_uss_ps_length_ui16(
        ps_correct_data.uss_ps_length_ui16);
    ps_correct_proto->set_uss_ps_angle_i16(
        ps_correct_data.uss_ps_angle_i16);

    ps_correct_proto->mutable_uss_ps_1st_corner()->set_x_si32(
        ps_correct_data.uss_ps_1st_corner.x_si32);
    ps_correct_proto->mutable_uss_ps_1st_corner()->set_y_si32(
        ps_correct_data.uss_ps_1st_corner.y_si32);

    ps_correct_proto->mutable_uss_ps_2nd_corner()->set_x_si32(
        ps_correct_data.uss_ps_2nd_corner.x_si32);
    ps_correct_proto->mutable_uss_ps_2nd_corner()->set_y_si32(
        ps_correct_data.uss_ps_2nd_corner.y_si32);

    ps_correct_proto->mutable_uss_ps_3st_corner()->set_x_si32(
        ps_correct_data.uss_ps_3st_corner.x_si32);
    ps_correct_proto->mutable_uss_ps_3st_corner()->set_y_si32(
        ps_correct_data.uss_ps_3st_corner.y_si32);

    ps_correct_proto->mutable_uss_ps_4nd_corner()->set_x_si32(
        ps_correct_data.uss_ps_4nd_corner.x_si32);
    ps_correct_proto->mutable_uss_ps_4nd_corner()->set_y_si32(
        ps_correct_data.uss_ps_4nd_corner.y_si32);

    ps_correct_proto->mutable_uss_ps_1st_corner_gcs()->set_x_si32(
        ps_correct_data.uss_ps_1st_corner_gcs.x_si32);
    ps_correct_proto->mutable_uss_ps_1st_corner_gcs()->set_y_si32(
        ps_correct_data.uss_ps_1st_corner_gcs.y_si32);

    ps_correct_proto->mutable_uss_ps_2nd_corner_gcs()->set_x_si32(
        ps_correct_data.uss_ps_2nd_corner_gcs.x_si32);
    ps_correct_proto->mutable_uss_ps_2nd_corner_gcs()->set_y_si32(
        ps_correct_data.uss_ps_2nd_corner_gcs.y_si32);

    ps_correct_proto->mutable_uss_ps_3st_corner_gcs()->set_x_si32(
        ps_correct_data.uss_ps_3st_corner_gcs.x_si32);
    ps_correct_proto->mutable_uss_ps_3st_corner_gcs()->set_y_si32(
        ps_correct_data.uss_ps_3st_corner_gcs.y_si32);

    ps_correct_proto->mutable_uss_ps_4nd_corner_gcs()->set_x_si32(
        ps_correct_data.uss_ps_4nd_corner_gcs.x_si32);
    ps_correct_proto->mutable_uss_ps_4nd_corner_gcs()->set_y_si32(
        ps_correct_data.uss_ps_4nd_corner_gcs.y_si32);

    ps_correct_proto->mutable_uss_ps_boundary_1st_info()->set_x_si32(
        ps_correct_data.uss_ps_boundary_1st_info.X_si16);
    ps_correct_proto->mutable_uss_ps_boundary_1st_info()->set_y_si32(
        ps_correct_data.uss_ps_boundary_1st_info.Y_si16);

    ps_correct_proto->mutable_uss_ps_boundary_2nd_info()->set_x_si32(
        ps_correct_data.uss_ps_boundary_2nd_info.X_si16);
    ps_correct_proto->mutable_uss_ps_boundary_2nd_info()->set_y_si32(
        ps_correct_data.uss_ps_boundary_2nd_info.Y_si16);

    ps_correct_proto->set_uss_ps_finish_timestamp(
        ps_correct_data.uss_ps_finish_timestamp);
  }  // end ps_correct

  return 0;
}

int UssTrackingModeDataSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& uss_tracking_mode_data{data_msg->MutableUssTrackingMode()};

  uss_tracking_mode_data.plv_mode = static_cast<PlvState>(proto_msg->plv_mode());
  uss_tracking_mode_data.psu_mode = static_cast<PsuState>(proto_msg->psu_mode());
  uss_tracking_mode_data.uss_ps_clear_distance = static_cast<std::uint8_t>(proto_msg->uss_ps_clear_distance());
  uss_tracking_mode_data.odo_mode = static_cast<OdoState>(proto_msg->odo_mode());

  {
    auto& ps_correct_data{uss_tracking_mode_data.ps_correct};
    auto& ps_correct_proto{proto_msg->ps_correct()};

    ps_correct_data.uss_ps_id_ui8 =
        static_cast<std::uint8_t>(ps_correct_proto.uss_ps_id_ui8());
    ps_correct_data.uss_ps_status_en =
        static_cast<UssParkingSlotsData::UssPsStatusEn>(
            ps_correct_proto.uss_ps_status_en());
    ps_correct_data.uss_ps_type_en =
        static_cast<UssParkingSlotsData::UssPsTypeEn>(
            ps_correct_proto.uss_ps_type_en());
    ps_correct_data.uss_ps_orientation_en =
        static_cast<UssParkingSlotsData::UssPsOrientationEn>(
            ps_correct_proto.uss_ps_orientation_en());

    ps_correct_data.uss_ps_1st_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            ps_correct_proto.uss_ps_1st_boundary_type_en());
    ps_correct_data.uss_ps_2nd_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            ps_correct_proto.uss_ps_2nd_boundary_type_en());
    ps_correct_data.uss_ps_cross_boundary_type_en =
        static_cast<UssParkingSlotsData::UssPsBoundaryTypeEn>(
            ps_correct_proto.uss_ps_cross_boundary_type_en());

    ps_correct_data.uss_ps_depth_ui16 =
        static_cast<std::uint16_t>(ps_correct_proto.uss_ps_depth_ui16());
    ps_correct_data.uss_ps_length_ui16 =
        static_cast<std::uint16_t>(ps_correct_proto.uss_ps_length_ui16());
    ps_correct_data.uss_ps_angle_i16 =
        static_cast<std::int16_t>(ps_correct_proto.uss_ps_angle_i16());

    if (ps_correct_proto.has_uss_ps_1st_corner()) {
      ps_correct_data.uss_ps_1st_corner.x_si32 =
          ps_correct_proto.uss_ps_1st_corner().x_si32();
      ps_correct_data.uss_ps_1st_corner.y_si32 =
          ps_correct_proto.uss_ps_1st_corner().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_2nd_corner()) {
      ps_correct_data.uss_ps_2nd_corner.x_si32 =
          ps_correct_proto.uss_ps_2nd_corner().x_si32();
      ps_correct_data.uss_ps_2nd_corner.y_si32 =
          ps_correct_proto.uss_ps_2nd_corner().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_3st_corner()) {
      ps_correct_data.uss_ps_3st_corner.x_si32 =
          ps_correct_proto.uss_ps_3st_corner().x_si32();
      ps_correct_data.uss_ps_3st_corner.y_si32 =
          ps_correct_proto.uss_ps_3st_corner().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_4nd_corner()) {
      ps_correct_data.uss_ps_4nd_corner.x_si32 =
          ps_correct_proto.uss_ps_4nd_corner().x_si32();
      ps_correct_data.uss_ps_4nd_corner.y_si32 =
          ps_correct_proto.uss_ps_4nd_corner().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_1st_corner_gcs()) {
      ps_correct_data.uss_ps_1st_corner_gcs.x_si32 =
          ps_correct_proto.uss_ps_1st_corner_gcs().x_si32();
      ps_correct_data.uss_ps_1st_corner_gcs.y_si32 =
          ps_correct_proto.uss_ps_1st_corner_gcs().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_2nd_corner_gcs()) {
      ps_correct_data.uss_ps_2nd_corner_gcs.x_si32 =
          ps_correct_proto.uss_ps_2nd_corner_gcs().x_si32();
      ps_correct_data.uss_ps_2nd_corner_gcs.y_si32 =
          ps_correct_proto.uss_ps_2nd_corner_gcs().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_3st_corner_gcs()) {
      ps_correct_data.uss_ps_3st_corner_gcs.x_si32 =
          ps_correct_proto.uss_ps_3st_corner_gcs().x_si32();
      ps_correct_data.uss_ps_3st_corner_gcs.y_si32 =
          ps_correct_proto.uss_ps_3st_corner_gcs().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_4nd_corner_gcs()) {
      ps_correct_data.uss_ps_4nd_corner_gcs.x_si32 =
          ps_correct_proto.uss_ps_4nd_corner_gcs().x_si32();
      ps_correct_data.uss_ps_4nd_corner_gcs.y_si32 =
          ps_correct_proto.uss_ps_4nd_corner_gcs().y_si32();
    }

    if (ps_correct_proto.has_uss_ps_boundary_1st_info()) {
      ps_correct_data.uss_ps_boundary_1st_info.X_si16 =
          static_cast<std::uint16_t>(
              ps_correct_proto.uss_ps_boundary_1st_info().x_si32());
      ps_correct_data.uss_ps_boundary_1st_info.Y_si16 =
          static_cast<std::uint16_t>(
              ps_correct_proto.uss_ps_boundary_1st_info().y_si32());
    }

    if (ps_correct_proto.has_uss_ps_boundary_2nd_info()) {
      ps_correct_data.uss_ps_boundary_2nd_info.X_si16 =
          static_cast<std::uint16_t>(
              ps_correct_proto.uss_ps_boundary_2nd_info().x_si32());
      ps_correct_data.uss_ps_boundary_2nd_info.Y_si16 =
          static_cast<std::uint16_t>(
              ps_correct_proto.uss_ps_boundary_2nd_info().y_si32());
    }

    ps_correct_data.uss_ps_finish_timestamp =
        ps_correct_proto.uss_ps_finish_timestamp();
  }  // end ps_correct

  return 0;
}

}  // namespace autodrive