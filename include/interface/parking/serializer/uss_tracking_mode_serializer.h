//
#ifndef AUTODRIVE_INTERFACE_USS_TRACKING_MODE_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_USS_TRACKING_MODE_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class UssTrackingModeMsg;
}  // namespace proto
}  // namespace workflow

class UssTrackingModeData;

class UssTrackingModeDataSerializer
    : public message::ProtobufSerializer<workflow::proto::UssTrackingModeMsg,
                                         UssTrackingModeData> {
 public:
  UssTrackingModeDataSerializer() noexcept = default;
  ~UssTrackingModeDataSerializer() noexcept = default;
  UssTrackingModeDataSerializer(UssTrackingModeDataSerializer const&) = default;
  UssTrackingModeDataSerializer& operator=(
      UssTrackingModeDataSerializer const&) = default;
  UssTrackingModeDataSerializer(UssTrackingModeDataSerializer&&) = default;
  UssTrackingModeDataSerializer& operator=(UssTrackingModeDataSerializer&&) =
      default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_USS_TRACKING_MODE_SERIALIZER_H_