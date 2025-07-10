//
#ifndef AUTODRIVE_INTERFACE_VPA_STATE_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_VPA_STATE_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class VpaStateMsg;
}  // namespace proto
}  // namespace workflow

class VpaStateData;

class VpaStateDataSerializer
    : public message::ProtobufSerializer<workflow::proto::VpaStateMsg,
                                         VpaStateData> {
 public:
  VpaStateDataSerializer() noexcept = default;
  ~VpaStateDataSerializer() noexcept = default;
  VpaStateDataSerializer(VpaStateDataSerializer const&) = default;
  VpaStateDataSerializer& operator=(VpaStateDataSerializer const&) = default;
  VpaStateDataSerializer(VpaStateDataSerializer&&) = default;
  VpaStateDataSerializer& operator=(VpaStateDataSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_VPA_STATE_SERIALIZER_H_