//
#include "model/serializer/camera_serializer.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "model/camera_data.h"
#include "protocol/camera.pb.h"

namespace autodrive {

int CameraSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  workflow::proto::Image* image{proto_msg->mutable_image()};
  CameraData::Image const& local_image{data_msg->GetImage()};
  image->set_width(local_image.width);
  image->set_height(local_image.height);
  image->set_element_size(local_image.element_size);
  if (local_image.data_ptr != nullptr) {
    if (local_image.data_size > 0U) {
      image->set_data(local_image.data_ptr, local_image.data_size);
    } else {
      image->set_data(local_image.data_ptr,
          local_image.width * local_image.height * local_image.element_size);
    }
  } else {
    if (local_image.data_size > 0U) {
      image->set_data(local_image.data.data(), local_image.data_size);
    } else {
      image->set_data(local_image.data.data(),
          local_image.width * local_image.height * local_image.element_size);
    }
  }
  image->set_pos(static_cast<workflow::proto::Position>(local_image.pos));
  image->set_mode(static_cast<workflow::proto::Mode>(local_image.mode));
  image->set_type(static_cast<workflow::proto::Type>(local_image.type));
  return 0;
}

int CameraSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  CameraData::Image& image{data_msg->MutableImage()};
  if (proto_msg->image().width() < 0 || proto_msg->image().height() < 0 ||
      proto_msg->image().element_size() < 0) {
    throw InvalidDataException("Invalid image size");
  }
  image.width = proto_msg->image().width();
  image.height = proto_msg->image().height();
  image.element_size = proto_msg->image().element_size();
  std::uint32_t size{static_cast<std::uint32_t>(image.width) *
      static_cast<std::uint32_t>(image.height) *
      static_cast<std::uint32_t>(image.element_size)};
  if (size != proto_msg->image().data().size()) {
    image.data_size = proto_msg->image().data().size();
    size = image.data_size;
  }
  if (size > CameraData::kMaxImageSize) {
    throw OutOfResourceException("Image too large");
  }
  memcpy(image.data.data(), proto_msg->image().data().c_str(), size);
  image.pos = static_cast<CameraData::Position>(proto_msg->image().pos());
  image.mode = static_cast<CameraData::Mode>(proto_msg->image().mode());
  image.type = static_cast<CameraData::Type>(proto_msg->image().type());
  return 0;
}

}