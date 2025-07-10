//
#ifndef AUTODRIVE_INTERFACE_FUSION_PARKING_SLOTS_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_FUSION_PARKING_SLOTS_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class FusionParkingSlotsMsg;
}  // namespace proto
}  // namespace workflow

class FusionParkingSlotsData;

class FusionParkingSlotsSerializer
    : public message::ProtobufSerializer<workflow::proto::FusionParkingSlotsMsg,
                                         FusionParkingSlotsData> {
 public:
  FusionParkingSlotsSerializer() noexcept = default;
  ~FusionParkingSlotsSerializer() noexcept = default;
  FusionParkingSlotsSerializer(FusionParkingSlotsSerializer const&) = default;
  FusionParkingSlotsSerializer& operator=(FusionParkingSlotsSerializer const&) = default;
  FusionParkingSlotsSerializer(FusionParkingSlotsSerializer&&) = default;
  FusionParkingSlotsSerializer& operator=(FusionParkingSlotsSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_FUSION_PARKING_SLOTS_SERIALIZER_H_