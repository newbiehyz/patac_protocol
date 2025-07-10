//
#include "interface/parking/serializer/image_serializer.h"

#include <utils/exception.h>

#include "interface/parking/image_data.h"
#include "protocol/image_data.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int ImageSerializer::Serialize(const message::SerializeMsg* data_ptr,
                               message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& image_data{data_msg->GetImageData()};

  workflow::proto::ImageInfo* image_Info_proto(proto_msg->mutable_image_info());
  image_Info_proto->set_width(image_data.image_info.width);
  image_Info_proto->set_height(image_data.image_info.height);

  workflow::proto::MemInfo* mem_info_y_proto{proto_msg->mutable_mem_info_y()};
  mem_info_y_proto->set_addr(image_data.mem_info_y.addr);
  mem_info_y_proto->set_pitch(image_data.mem_info_y.pitch);

  workflow::proto::MemInfo* mem_info_uv_proto(proto_msg->mutable_mem_info_uv());
  mem_info_uv_proto->set_addr(image_data.mem_info_uv.addr);
  mem_info_uv_proto->set_size(image_data.mem_info_uv.size);
  mem_info_uv_proto->set_pitch(image_data.mem_info_uv.pitch);

  return 0;
}

int ImageSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                 message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& image_data{data_msg->MutableImageData()};

  auto& image_info_proto{proto_msg->image_info()};
  image_data.image_info.height = image_info_proto.height();
  image_data.image_info.width = image_info_proto.width();

  auto& mem_info_y_proto{proto_msg->mem_info_y()};
  image_data.mem_info_y.addr = mem_info_y_proto.addr();
  image_data.mem_info_y.size = mem_info_y_proto.size();
  image_data.mem_info_y.pitch = mem_info_y_proto.pitch();

  auto& mem_info_uv_proto{proto_msg->mem_info_uv()};
  image_data.mem_info_uv.addr = mem_info_uv_proto.addr();
  image_data.mem_info_uv.size = mem_info_uv_proto.size();
  image_data.mem_info_uv.pitch = mem_info_uv_proto.pitch();

  return 0;
}

}  // namespace autodrive