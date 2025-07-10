//
#ifndef AUTODRIVE_INTERFACE_IMAGE_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_IMAGE_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ImageMsg;
}  // namespace proto
}  // namespace workflow

class ImageData;

class ImageSerializer
    : public message::ProtobufSerializer<workflow::proto::ImageMsg, ImageData> {
 public:
  ImageSerializer() noexcept = default;
  ~ImageSerializer() noexcept = default;
  ImageSerializer(ImageSerializer const&) = default;
  ImageSerializer& operator=(ImageSerializer const&) = default;
  ImageSerializer(ImageSerializer&&) = default;
  ImageSerializer& operator=(ImageSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_IMAGE_SERIALIZER_H_