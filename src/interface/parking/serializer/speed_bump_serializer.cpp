//
#include "interface/parking/serializer/speed_bump_serializer.h"

#include <utils/exception.h>

#include "interface/parking/speed_bump_data.h"
#include "protocol/speed_bump.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int SpeedBumpSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& speed_bumps{data_msg->GetSpeedBumps()};

  auto speed_bump_nums = speed_bumps.speed_bump_nums;
  proto_msg->set_speed_bump_nums(speed_bump_nums);

  {
    auto& sb_list = speed_bumps.speed_bump_list;
    for (std::int32_t sb_idx = 0; sb_idx < speed_bump_nums; sb_idx++) {
      auto& sb_data{sb_list.at(sb_idx)};
      workflow::proto::SpeedBumps* sb_proto = proto_msg->add_speed_bump_list();
      sb_proto->set_speed_bump_id(sb_data.speed_bump_id);
      workflow::proto::Vector2f* p_0_vcs_proto = sb_proto->mutable_p_0_vcs();
      p_0_vcs_proto->set_x(sb_data.p_0_vcs.x);
      p_0_vcs_proto->set_y(sb_data.p_0_vcs.y);
      workflow::proto::Vector2f* p_1_vcs_proto = sb_proto->mutable_p_1_vcs();
      p_1_vcs_proto->set_x(sb_data.p_1_vcs.x);
      p_1_vcs_proto->set_y(sb_data.p_1_vcs.y);
      workflow::proto::Vector2f* p_2_vcs_proto = sb_proto->mutable_p_2_vcs();
      p_2_vcs_proto->set_x(sb_data.p_2_vcs.x);
      p_2_vcs_proto->set_y(sb_data.p_2_vcs.y);
      workflow::proto::Vector2f* p_3_vcs_proto = sb_proto->mutable_p_3_vcs();
      p_3_vcs_proto->set_x(sb_data.p_3_vcs.x);
      p_3_vcs_proto->set_y(sb_data.p_3_vcs.y);
    }
  }

  return 0;
}

int SpeedBumpSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                      message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto& speed_bumps{data_msg->MutableSpeedBumps()};
  auto sb_nums = proto_msg->speed_bump_nums();
  speed_bumps.speed_bump_nums = sb_nums;

  {
    if (sb_nums > SpeedBumpData::kMaxSpeedBumpSize) {
      throw OutOfResourceException("speed bump out of range");
    }
    auto sb_list = speed_bumps.speed_bump_list;
    for (std::int32_t sb_idx = 0; sb_idx < sb_nums; sb_idx++) {
      auto& speed_bump{sb_list.at(sb_idx)};
      speed_bump.speed_bump_id = proto_msg->speed_bump_list(sb_idx).speed_bump_id();
      speed_bump.p_0_vcs.x = proto_msg->speed_bump_list(sb_idx).p_0_vcs().x();
      speed_bump.p_0_vcs.y = proto_msg->speed_bump_list(sb_idx).p_0_vcs().y();
      speed_bump.p_1_vcs.x = proto_msg->speed_bump_list(sb_idx).p_1_vcs().x();
      speed_bump.p_1_vcs.y = proto_msg->speed_bump_list(sb_idx).p_1_vcs().y();
      speed_bump.p_2_vcs.x = proto_msg->speed_bump_list(sb_idx).p_2_vcs().x();
      speed_bump.p_2_vcs.y = proto_msg->speed_bump_list(sb_idx).p_2_vcs().y();
      speed_bump.p_3_vcs.x = proto_msg->speed_bump_list(sb_idx).p_3_vcs().x();
      speed_bump.p_3_vcs.y = proto_msg->speed_bump_list(sb_idx).p_3_vcs().y();
    }
  } 

  return 0;
}

}  // namespace autodrive