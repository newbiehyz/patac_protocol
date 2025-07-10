//
#include "model/serializer/gnss_serializer.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "model/gnss_data.h"
#include "protocol/gnss.pb.h"

namespace autodrive {

int GnssSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto &proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto &data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  workflow::proto::Gnss* gnss{proto_msg->mutable_gnss()};
  gnss->set_utc_time(data_msg->GetGnss().utc_time);
  gnss->set_latitude(data_msg->GetGnss().latitude);
  gnss->set_longitude(data_msg->GetGnss().longitude);
  gnss->set_altitude(data_msg->GetGnss().altitude);
  gnss->set_ellipsoid(data_msg->GetGnss().ellipsoid);
  gnss->set_pose_type(static_cast<workflow::proto::PoseType>(data_msg->GetGnss().pose_type));
  gnss->set_position_status(static_cast<workflow::proto::PositionStatus>(data_msg->GetGnss().position_status));
  gnss->set_num_satellites(data_msg->GetGnss().num_satellites);
  gnss->set_gnss_vel_north(data_msg->GetGnss().gnss_vel_north);
  gnss->set_gnss_vel_east(data_msg->GetGnss().gnss_vel_east);
  gnss->set_gnss_vel_down(data_msg->GetGnss().gnss_vel_down);
  gnss->set_heading(data_msg->GetGnss().heading);
  gnss->set_course(data_msg->GetGnss().course);
  gnss->set_heading_error(data_msg->GetGnss().heading_error);
  gnss->set_course_error(data_msg->GetGnss().course_error);
  gnss->set_horizon_position_error(data_msg->GetGnss().horizon_position_error);
  gnss->set_vertical_position_error(data_msg->GetGnss().vertical_position_error);
  gnss->set_horizon_velocity_error(data_msg->GetGnss().horizon_velocity_error);
  gnss->set_vertical_velocity_error(data_msg->GetGnss().vertical_velocity_error);
  gnss->set_tdop(data_msg->GetGnss().tdop);
  gnss->set_hdop(data_msg->GetGnss().hdop);
  gnss->set_vdop(data_msg->GetGnss().vdop);
  gnss->set_sat_seq_no(data_msg->GetGnss().sat_seq_no);
  gnss->set_num_gnss_satellite_info(data_msg->GetGnss().num_gnss_satellite_info);
  for (std::uint8_t index{0U}; index < data_msg->GetGnss().num_gnss_satellite_info; ++index) {
    workflow::proto::SatelliteInfo* si{gnss->add_statelline_info()};
    auto& l_si{data_msg->GetGnss().statelline_info.at(index)};
    si->set_num_in_view(l_si.num_in_view);
    si->set_no(l_si.no);
    si->set_snr(l_si.snr);
    si->set_elevation(l_si.elevation);
    si->set_azimuth(l_si.azimuth);
    si->set_positing_system(static_cast<workflow::proto::PositingSystem>(l_si.positing_system));
  }
  return 0;
}

int GnssSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  GnssData::Gnss& gnss{data_msg->MutableGnss()};
  gnss.utc_time = proto_msg->gnss().utc_time();
  gnss.latitude = proto_msg->gnss().latitude();
  gnss.longitude = proto_msg->gnss().longitude();
  gnss.altitude = proto_msg->gnss().altitude();
  gnss.ellipsoid = proto_msg->gnss().ellipsoid();
  gnss.pose_type = static_cast<
      GnssData::PoseType>(proto_msg->gnss().pose_type());
  gnss.position_status = static_cast<
      GnssData::PositionStatus>(proto_msg->gnss().position_status());
  gnss.num_satellites = proto_msg->gnss().num_satellites();
  gnss.gnss_vel_north = proto_msg->gnss().gnss_vel_north();
  gnss.gnss_vel_east = proto_msg->gnss().gnss_vel_east();
  gnss.gnss_vel_down = proto_msg->gnss().gnss_vel_down();
  gnss.heading = proto_msg->gnss().heading();
  gnss.course = proto_msg->gnss().course();
  gnss.heading_error = proto_msg->gnss().heading_error();
  gnss.course_error = proto_msg->gnss().course_error();
  gnss.horizon_position_error = proto_msg->gnss().horizon_position_error();
  gnss.vertical_position_error = proto_msg->gnss().vertical_position_error();
  gnss.horizon_velocity_error = proto_msg->gnss().horizon_velocity_error();
  gnss.vertical_velocity_error = proto_msg->gnss().vertical_velocity_error();
  gnss.tdop = proto_msg->gnss().tdop();
  gnss.hdop = proto_msg->gnss().hdop();
  gnss.vdop = proto_msg->gnss().vdop();
  gnss.sat_seq_no = proto_msg->gnss().sat_seq_no();
  gnss.num_gnss_satellite_info = static_cast<std::uint8_t>(proto_msg->gnss().num_gnss_satellite_info());
  if (static_cast<std::uint32_t>(gnss.num_gnss_satellite_info) >
      GnssData::kMaxSatInfoNum) {
    throw OutOfResourceException("Gnss statelline info size too long");
  }
 
  for (std::uint8_t i{0U}; i < gnss.num_gnss_satellite_info; ++i) {
    gnss.statelline_info.at(i).num_in_view = proto_msg->gnss().statelline_info(i).num_in_view();
    gnss.statelline_info.at(i).no = proto_msg->gnss().statelline_info(i).no();
    gnss.statelline_info.at(i).snr = proto_msg->gnss().statelline_info(i).snr();
    gnss.statelline_info.at(i).elevation = proto_msg->gnss().statelline_info(i).elevation();
    gnss.statelline_info.at(i).azimuth = proto_msg->gnss().statelline_info(i).azimuth();
    gnss.statelline_info.at(i).positing_system = static_cast<
        GnssData::PositingSystem>(proto_msg->gnss().statelline_info(i).positing_system());
  }

  return 0;
}

}