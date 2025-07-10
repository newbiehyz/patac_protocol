//
#include "interface/parking/serializer/lane_line_serializer.h"

#include <utils/exception.h>

#include "interface/parking/lane_line_data.h"
#include "protocol/lane_line.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int LaneLineSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  const auto& llp_data = data_msg->GetLaneLinePoints();
  auto* llp_proto = proto_msg->mutable_lane_line_points();

  auto* n_proto = llp_proto->mutable_near_line();
  n_proto->set_flag(llp_data.near_line.flag);
  auto* n_s_proto = n_proto->mutable_line_start_point_vcs();
  n_s_proto->set_x(llp_data.near_line.line_start_point_vcs.x);
  n_s_proto->set_y(llp_data.near_line.line_start_point_vcs.y);
  auto* n_e_proto = n_proto->mutable_line_end_point_vcs();
  n_e_proto->set_x(llp_data.near_line.line_end_point_vcs.x);
  n_e_proto->set_y(llp_data.near_line.line_end_point_vcs.y);

  auto* f_proto = llp_proto->mutable_far_line();
  f_proto->set_flag(llp_data.far_line.flag);
  auto* f_s_proto = f_proto->mutable_line_start_point_vcs();
  f_s_proto->set_x(llp_data.far_line.line_start_point_vcs.x);
  f_s_proto->set_y(llp_data.far_line.line_start_point_vcs.y);
  auto* f_e_proto = f_proto->mutable_line_end_point_vcs();
  f_e_proto->set_x(llp_data.far_line.line_end_point_vcs.x);
  f_e_proto->set_y(llp_data.far_line.line_end_point_vcs.y);

  return 0;
}

int LaneLineSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                      message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto& llp_data = data_msg->MutableLaneLinePoints();
  auto& llp_proto = proto_msg->lane_line_points();

  auto& n_proto = llp_proto.near_line();
  auto& n_data = llp_data.near_line;
  n_data.flag = n_proto.flag();
  n_data.line_start_point_vcs.x = n_proto.line_start_point_vcs().x();
  n_data.line_start_point_vcs.y = n_proto.line_start_point_vcs().y();
  n_data.line_end_point_vcs.x = n_proto.line_end_point_vcs().x();
  n_data.line_end_point_vcs.y = n_proto.line_end_point_vcs().y();

  auto& f_proto = llp_proto.far_line();
  auto& f_data = llp_data.far_line;
  f_data.flag = f_proto.flag();
  f_data.line_start_point_vcs.x = f_proto.line_start_point_vcs().x();
  f_data.line_start_point_vcs.y = f_proto.line_start_point_vcs().y();
  f_data.line_end_point_vcs.x = f_proto.line_end_point_vcs().x();
  f_data.line_end_point_vcs.y = f_proto.line_end_point_vcs().y();

  return 0;
}

}  // namespace autodrive