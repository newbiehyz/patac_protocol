//
#ifndef AUTODRIVE_INTERFACE_VISION_SYSTEM_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_VISION_SYSTEM_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class VisionSystemMsg;
}  // namespace proto
}  // namespace workflow

class VisionSystemData;

class VisionSystemSerializer
    : public message::ProtobufSerializer<workflow::proto::VisionSystemMsg,
                                         VisionSystemData> {
 public:
  VisionSystemSerializer() noexcept = default;
  ~VisionSystemSerializer() noexcept = default;
  VisionSystemSerializer(VisionSystemSerializer const&) = default;
  VisionSystemSerializer& operator=(VisionSystemSerializer const&) = default;
  VisionSystemSerializer(VisionSystemSerializer&&) = default;
  VisionSystemSerializer& operator=(VisionSystemSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_VISION_SYSTEM_SERIALIZER_H_