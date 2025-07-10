//
#ifndef AUTODRIVE_INTERFACE_USS_SLOTS_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_USS_SLOTS_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class UssParkingSlotsMsg;
}  // namespace proto
}  // namespace workflow

class UssParkingSlotsData;

class UssParkingSlotsDataSerializer
    : public message::ProtobufSerializer<workflow::proto::UssParkingSlotsMsg,
                                         UssParkingSlotsData> {
 public:
  UssParkingSlotsDataSerializer() noexcept = default;
  ~UssParkingSlotsDataSerializer() noexcept = default;
  UssParkingSlotsDataSerializer(UssParkingSlotsDataSerializer const&) = default;
  UssParkingSlotsDataSerializer& operator=(
      UssParkingSlotsDataSerializer const&) = default;
  UssParkingSlotsDataSerializer(UssParkingSlotsDataSerializer&&) = default;
  UssParkingSlotsDataSerializer& operator=(UssParkingSlotsDataSerializer&&) =
      default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_USS_SLOTS_SERIALIZER_H_