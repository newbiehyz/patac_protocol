//
#include "model/serializer/camera_extrinsics_serializer.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "model/camera_extrinsics_data.h"
#include "protocol/camera_extrinsics.pb.h"

namespace autodrive {

int CameraExtrinsicsSerializer::Serialize(const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  for (std::uint8_t index{0U}; index < data_msg->GetExtrinsics().num_extrinsics; ++index) {
    workflow::proto::Extrinsic* extrinsic{proto_msg->add_extrinsics()};
    auto& ex{data_msg->GetExtrinsics().extrinsics.at(index)};
    extrinsic->set_x(ex.x);
    extrinsic->set_y(ex.y);
    extrinsic->set_z(ex.z);
    extrinsic->set_roll(ex.roll);
    extrinsic->set_pitch(ex.pitch);
    extrinsic->set_yaw(ex.yaw);
    extrinsic->set_num_extrinsic(ex.num_extrinsic);
    extrinsic->set_cam_id(static_cast<workflow::proto::CameraID>(ex.cam_id));
  }
  return 0;
}

int CameraExtrinsicsSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& extrinsics{data_msg->MutableExtrinsics()};
  extrinsics.num_extrinsics = static_cast<std::uint8_t>(proto_msg->extrinsics_size());
  if (static_cast<std::uint32_t>(proto_msg->extrinsics_size()) > CameraExtrinsicsData::kMaxExtrinsicsSize) {
    throw OutOfResourceException("Camera extrinsics size too long");
  }
  for (std::uint8_t i{0U}; i < extrinsics.num_extrinsics; ++i) {
    auto& ex_data{extrinsics.extrinsics.at(i)};
    auto &ex_msg{proto_msg->extrinsics(i)};
    ex_data.x = ex_msg.x();
    ex_data.y = ex_msg.y();
    ex_data.z = ex_msg.z();
    ex_data.roll = ex_msg.roll();
    ex_data.pitch = ex_msg.pitch();
    ex_data.yaw = ex_msg.yaw();
    ex_data.num_extrinsic = ex_msg.num_extrinsic();
    ex_data.cam_id = static_cast<CameraID>(ex_msg.cam_id());
  }
  return 0;
}

}