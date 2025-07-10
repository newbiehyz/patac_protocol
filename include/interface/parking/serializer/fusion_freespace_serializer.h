//
#ifndef AUTODRIVE_INTERFACE_FUSION_FREESPACE_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_FUSION_FREESPACE_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class FreespaceFusionMsg;
}  // namespace proto
}  // namespace workflow

class FusionFreespaceData;

class FusionFreespaceSerializer
    : public message::ProtobufSerializer<workflow::proto::FreespaceFusionMsg,
                                         FusionFreespaceData> {
 public:
  FusionFreespaceSerializer() noexcept = default;
  ~FusionFreespaceSerializer() noexcept = default;
  FusionFreespaceSerializer(FusionFreespaceSerializer const&) = default;
  FusionFreespaceSerializer& operator=(FusionFreespaceSerializer const&) = default;
  FusionFreespaceSerializer(FusionFreespaceSerializer&&) = default;
  FusionFreespaceSerializer& operator=(FusionFreespaceSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_FUSION_FREESPACE_SERIALIZER_H_