//
#include "model/serializer/camera_intrinsics_serializer.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "model/camera_intrinsics_data.h"
#include "protocol/camera_intrinsics.pb.h"

namespace autodrive {

int CameraIntrinsicsSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  for (std::uint8_t index{0U}; index < data_msg->GetIntrinsics().num_intrinsics; ++index) {
    workflow::proto::Intrinsic* intrinsic{proto_msg->add_intrinsics()};
    auto& in{data_msg->GetIntrinsics().intrinsics.at(index)};
    intrinsic->set_cx(in.cx);
    intrinsic->set_cy(in.cy);
    intrinsic->set_fx(in.fx);
    intrinsic->set_fy(in.fy);
    intrinsic->set_cod_x(in.cod_x);
    intrinsic->set_cod_y(in.cod_y);
    intrinsic->set_cam_id(static_cast<workflow::proto::CameraID>(in.cam_id));

    for (std::uint32_t k_index = 0; k_index < in.num_k; ++k_index) {
      intrinsic->add_k(in.k.at(k_index));
    }
    for (std::uint32_t p_index = 0; p_index < in.num_p; ++p_index) {
      intrinsic->add_p(in.p.at(p_index));
    }
    for (std::uint32_t s_index = 0; s_index < in.num_s; ++s_index) {
      intrinsic->add_s(in.s.at(s_index));
    }
  }
  return 0;
}

int CameraIntrinsicsSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& intrinsics{data_msg->MutableIntrinsics()};
  intrinsics.num_intrinsics = static_cast<std::uint8_t>(proto_msg->intrinsics_size());
  if (static_cast<std::uint32_t>(proto_msg->intrinsics_size()) > CameraIntrinsicsData::kMaxCameraSize) {
    throw OutOfResourceException("Camera intrinsics size too long");
  }
  for (std::uint8_t i{0U}; i < intrinsics.num_intrinsics; ++i) {
    intrinsics.intrinsics.at(i).cx = proto_msg->intrinsics(i).cx();
    intrinsics.intrinsics.at(i).cy = proto_msg->intrinsics(i).cy();
    intrinsics.intrinsics.at(i).fx = proto_msg->intrinsics(i).fx();
    intrinsics.intrinsics.at(i).fy = proto_msg->intrinsics(i).fy();
    intrinsics.intrinsics.at(i).cod_x = proto_msg->intrinsics(i).cod_x();
    intrinsics.intrinsics.at(i).cod_y = proto_msg->intrinsics(i).cod_y();
    intrinsics.intrinsics.at(i).cam_id =
        static_cast<CameraID>(proto_msg->intrinsics(i).cam_id());

    intrinsics.intrinsics.at(i).num_k = proto_msg->intrinsics(i).k_size();
    if (static_cast<std::uint32_t>(proto_msg->intrinsics(i).k_size()) >
        CameraIntrinsicsData::kMaxkSize) {
      throw OutOfResourceException("Camera intrinsics k size too long");
    }
    intrinsics.intrinsics.at(i).num_p = proto_msg->intrinsics(i).p_size();
    if (static_cast<std::uint32_t>(proto_msg->intrinsics(i).p_size()) >
        CameraIntrinsicsData::kMaxpSize) {
      throw OutOfResourceException("Camera intrinsics p size too long");
    }
    intrinsics.intrinsics.at(i).num_s = proto_msg->intrinsics(i).s_size();
    if (static_cast<std::uint32_t>(proto_msg->intrinsics(i).s_size()) >
        CameraIntrinsicsData::kMaxsSize) {
      throw OutOfResourceException("Camera intrinsics s size too long");
    }

    for (uint32_t k_index = 0; k_index < intrinsics.intrinsics.at(i).num_k;
         ++k_index) {
      intrinsics.intrinsics.at(i).k.at(k_index) =
          proto_msg->intrinsics(i).k(k_index);
    }
    for (uint32_t p_index = 0; p_index < intrinsics.intrinsics.at(i).num_p;
         ++p_index) {
      intrinsics.intrinsics.at(i).p.at(p_index) =
          proto_msg->intrinsics(i).p(p_index);
    }
    for (uint32_t s_index = 0; s_index < intrinsics.intrinsics.at(i).num_s;
         ++s_index) {
      intrinsics.intrinsics.at(i).s.at(s_index) =
          proto_msg->intrinsics(i).s(s_index);
    }
  }
  return 0;
}

}