//
#ifndef AUTODRIVE_INTERFACE_LOCALIZATION_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_LOCALIZATION_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class LocalizationMsg;
}  // namespace proto
}  // namespace workflow

class LocalizationData;

class LocalizationSerializer
    : public message::ProtobufSerializer<workflow::proto::LocalizationMsg,
                                         LocalizationData> {
 public:
  LocalizationSerializer() noexcept = default;
  ~LocalizationSerializer() noexcept = default;
  LocalizationSerializer(LocalizationSerializer const&) = default;
  LocalizationSerializer& operator=(LocalizationSerializer const&) = default;
  LocalizationSerializer(LocalizationSerializer&&) = default;
  LocalizationSerializer& operator=(LocalizationSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_LOCALIZATION_SERIALIZER_H_