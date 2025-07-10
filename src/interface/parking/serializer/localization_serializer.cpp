//
#include "interface/parking/serializer/localization_serializer.h"

#include <utils/exception.h>

#include "interface/parking/localization_data.h"
#include "protocol/localization.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int LocalizationSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& local_data{data_msg->GetLocalization()};

  workflow::proto::ndm_base::NdmSpec* ndm_proto =
      proto_msg->mutable_ndm_header();
  auto& ndm_data{local_data.ndm_header};
  ndm_proto->set_version(ndm_data.version);
  ndm_proto->set_date(ndm_data.date);
  ndm_proto->set_timestamp(ndm_data.timestamp);
  ndm_proto->set_seq_id(ndm_data.seq_id);
  ndm_proto->set_projection(ndm_data.projection);
  ndm_proto->set_utm_zone(ndm_data.utm_zone);
  ndm_proto->mutable_base_offset()->set_x(ndm_data.base_offset.x);
  ndm_proto->mutable_base_offset()->set_y(ndm_data.base_offset.y);
  ndm_proto->mutable_base_offset()->set_z(ndm_data.base_offset.z);

  workflow::proto::SpLocation* splical_proto = proto_msg->mutable_location();
  auto& splocal_data{local_data.location};
  {
    workflow::proto::loc_odo::Odometry* loc_odom_proto =
        splical_proto->mutable_loc_odom();
    auto& loc_odom_data{splocal_data.loc_odom};
    loc_odom_proto->set_stamp(loc_odom_data.stamp);
    loc_odom_proto->mutable_quaternion()->set_x(loc_odom_data.quaternion.x);
    loc_odom_proto->mutable_quaternion()->set_y(loc_odom_data.quaternion.y);
    loc_odom_proto->mutable_quaternion()->set_z(loc_odom_data.quaternion.z);
    loc_odom_proto->mutable_quaternion()->set_w(loc_odom_data.quaternion.w);

    loc_odom_proto->mutable_position()->set_x(loc_odom_data.position.x);
    loc_odom_proto->mutable_position()->set_y(loc_odom_data.position.y);
    loc_odom_proto->mutable_position()->set_z(loc_odom_data.position.z);

    loc_odom_proto->set_seq_id(loc_odom_data.seq_id);

    for (std::uint32_t pc_idx = 0; pc_idx < loc_odom_data.num_pose_cov;
         pc_idx++) {
      loc_odom_proto->add_pose_cov(loc_odom_data.pose_cov.at(pc_idx));
    }

    loc_odom_proto->mutable_linear_velocity()->set_x(
        loc_odom_data.linear_velocity.x);
    loc_odom_proto->mutable_linear_velocity()->set_y(
        loc_odom_data.linear_velocity.y);
    loc_odom_proto->mutable_linear_velocity()->set_z(
        loc_odom_data.linear_velocity.z);

    loc_odom_proto->mutable_angular_velocity()->set_x(
        loc_odom_data.angular_velocity.x);
    loc_odom_proto->mutable_angular_velocity()->set_y(
        loc_odom_data.angular_velocity.y);
    loc_odom_proto->mutable_angular_velocity()->set_z(
        loc_odom_data.angular_velocity.z);

    for (std::uint32_t tc_idx = 0; tc_idx < loc_odom_data.num_twist_cov;
         tc_idx++) {
      loc_odom_proto->add_twist_cov(loc_odom_data.twist_cov.at(tc_idx));
    }

    loc_odom_proto->mutable_map_offset()->set_x(loc_odom_data.map_offset.x);
    loc_odom_proto->mutable_map_offset()->set_y(loc_odom_data.map_offset.y);
    loc_odom_proto->mutable_map_offset()->set_z(loc_odom_data.map_offset.z);

    loc_odom_proto->mutable_velocity()->set_x(loc_odom_data.velocity.x);
    loc_odom_proto->mutable_velocity()->set_y(loc_odom_data.velocity.y);
    loc_odom_proto->mutable_velocity()->set_z(loc_odom_data.velocity.z);

    loc_odom_proto->mutable_acceleration()->set_x(loc_odom_data.acceleration.x);
    loc_odom_proto->mutable_acceleration()->set_y(loc_odom_data.acceleration.y);
    loc_odom_proto->mutable_acceleration()->set_z(loc_odom_data.acceleration.z);
    loc_odom_proto->set_floor(loc_odom_data.floor);
  }  // loc_odom
  splical_proto->set_state(
      static_cast<workflow::proto::SpLocation_SpLocationState>(
          splocal_data.state));
  splical_proto->mutable_velocity()->set_x(splocal_data.velocity.x);
  splical_proto->mutable_velocity()->set_y(splocal_data.velocity.y);
  splical_proto->mutable_velocity()->set_z(splocal_data.velocity.z);

  splical_proto->mutable_angular_vel()->set_x(splocal_data.angular_vel.x);
  splical_proto->mutable_angular_vel()->set_y(splocal_data.angular_vel.y);
  splical_proto->mutable_angular_vel()->set_z(splocal_data.angular_vel.z);

  return 0;
}

int LocalizationSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& local_data{data_msg->MutableLocalization()};

  if (proto_msg->has_ndm_header()) {
    auto& ndm_proto{proto_msg->ndm_header()};
    auto& ndm_data{local_data.ndm_header};
    memcpy(ndm_data.version, ndm_proto.version().c_str(),
           ndm_proto.version().length());
    memcpy(ndm_data.date, ndm_proto.date().c_str(), ndm_proto.date().length());
    ndm_data.timestamp = ndm_proto.timestamp();
    ndm_data.seq_id = ndm_proto.seq_id();
    memcpy(ndm_data.projection, ndm_proto.projection().c_str(),
           ndm_proto.projection().length());
    memcpy(ndm_data.utm_zone, ndm_proto.utm_zone().c_str(),
           ndm_proto.utm_zone().length());
    ndm_data.base_offset.x = ndm_proto.base_offset().x();
    ndm_data.base_offset.y = ndm_proto.base_offset().y();
    ndm_data.base_offset.z = ndm_proto.base_offset().z();
  }  // end ndm_header

  if (proto_msg->has_location()) {
    auto& splocal_proto{proto_msg->location()};
    auto& splocal_data{local_data.location};
    if (splocal_proto.has_loc_odom()) {
      auto& odom_proto{splocal_proto.loc_odom()};
      auto& odom_data{splocal_data.loc_odom};
      odom_data.stamp = odom_proto.stamp();
      if (odom_proto.has_quaternion()) {
        odom_data.quaternion.w = odom_proto.quaternion().w();
        odom_data.quaternion.x = odom_proto.quaternion().x();
        odom_data.quaternion.y = odom_proto.quaternion().y();
        odom_data.quaternion.z = odom_proto.quaternion().z();
      }
      if (odom_proto.has_position()) {
        odom_data.position.x = odom_proto.position().x();
        odom_data.position.y = odom_proto.position().y();
        odom_data.position.z = odom_proto.position().z();
      }
      odom_data.seq_id = odom_proto.seq_id();

      odom_data.num_pose_cov = odom_proto.pose_cov_size();
      for (std::uint32_t pc_idx = 0; pc_idx < odom_data.num_pose_cov;
           pc_idx++) {
        odom_data.pose_cov.at(pc_idx) = odom_proto.pose_cov(pc_idx);
      }

      odom_data.linear_velocity.x = odom_proto.linear_velocity().x();
      odom_data.linear_velocity.y = odom_proto.linear_velocity().y();
      odom_data.linear_velocity.z = odom_proto.linear_velocity().z();

      odom_data.angular_velocity.x = odom_proto.angular_velocity().x();
      odom_data.angular_velocity.y = odom_proto.angular_velocity().y();
      odom_data.angular_velocity.z = odom_proto.angular_velocity().z();

      odom_data.num_twist_cov = odom_proto.twist_cov_size();
      for (std::uint32_t tc_idx = 0; tc_idx < odom_data.num_twist_cov;
           tc_idx++) {
        odom_data.twist_cov.at(tc_idx) = odom_proto.twist_cov(tc_idx);
      }

      odom_data.map_offset.x = odom_proto.map_offset().x();
      odom_data.map_offset.y = odom_proto.map_offset().y();
      odom_data.map_offset.z = odom_proto.map_offset().z();

      odom_data.velocity.x = odom_proto.velocity().x();
      odom_data.velocity.y = odom_proto.velocity().y();
      odom_data.velocity.z = odom_proto.velocity().z();

      odom_data.acceleration.x = odom_proto.acceleration().x();
      odom_data.acceleration.y = odom_proto.acceleration().y();
      odom_data.acceleration.z = odom_proto.acceleration().z();
      odom_data.floor = odom_proto.floor();
    }  // end loc_odom

    splocal_data.state =
        static_cast<LocalizationData::SpLocation::SpLocationState>(
            splocal_proto.state());
  
    splocal_data.velocity.x = splocal_proto.velocity().x();
    splocal_data.velocity.y = splocal_proto.velocity().y();
    splocal_data.velocity.z = splocal_proto.velocity().z();

    splocal_data.angular_vel.x = splocal_proto.angular_vel().x();
    splocal_data.angular_vel.y = splocal_proto.angular_vel().y();
    splocal_data.angular_vel.z = splocal_proto.angular_vel().z();
  }  // end location

  return 0;
}

}  // namespace autodrive