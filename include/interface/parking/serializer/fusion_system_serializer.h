//
#ifndef AUTODRIVE_INTERFACE_FUSION_SYSTEM_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_FUSION_SYSTEM_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class FusionSystemMsg;
}  // namespace proto
}  // namespace workflow

class FusionSystemData;

class FusionSystemSerializer
    : public message::ProtobufSerializer<workflow::proto::FusionSystemMsg,
                                         FusionSystemData> {
 public:
  FusionSystemSerializer() noexcept = default;
  ~FusionSystemSerializer() noexcept = default;
  FusionSystemSerializer(FusionSystemSerializer const&) = default;
  FusionSystemSerializer& operator=(FusionSystemSerializer const&) = default;
  FusionSystemSerializer(FusionSystemSerializer&&) = default;
  FusionSystemSerializer& operator=(FusionSystemSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_FUSION_SYSTEM_SERIALIZER_H_