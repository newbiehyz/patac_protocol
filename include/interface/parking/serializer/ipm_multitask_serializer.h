//
#ifndef AUTODRIVE_INTERFACE_IPM_MULTITASK_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_IPM_MULTITASK_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class IpmMultitaskMsg;
}  // namespace proto
}  // namespace workflow

class IpmMultitaskData;

class IpmMultitaskSerializer
    : public message::ProtobufSerializer<workflow::proto::IpmMultitaskMsg,
                                         IpmMultitaskData> {
 public:
  IpmMultitaskSerializer() noexcept = default;
  ~IpmMultitaskSerializer() noexcept = default;
  IpmMultitaskSerializer(IpmMultitaskSerializer const&) = default;
  IpmMultitaskSerializer& operator=(IpmMultitaskSerializer const&) = default;
  IpmMultitaskSerializer(IpmMultitaskSerializer&&) = default;
  IpmMultitaskSerializer& operator=(IpmMultitaskSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_VISION_SYSTEM_SERIALIZER_H_