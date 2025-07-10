//
#ifndef AUTODRIVE_INTERFACE_PARKING_SLOTS_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_PARKING_SLOTS_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ParkingSlotsMsg;
}  // namespace proto
}  // namespace workflow

class ParkingSlotsData;

class ParkingSlotsSerializer
    : public message::ProtobufSerializer<workflow::proto::ParkingSlotsMsg,
                                         ParkingSlotsData> {
 public:
  ParkingSlotsSerializer() noexcept = default;
  ~ParkingSlotsSerializer() noexcept = default;
  ParkingSlotsSerializer(ParkingSlotsSerializer const&) = default;
  ParkingSlotsSerializer& operator=(ParkingSlotsSerializer const&) = default;
  ParkingSlotsSerializer(ParkingSlotsSerializer&&) = default;
  ParkingSlotsSerializer& operator=(ParkingSlotsSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_PARKING_SLOTS_SERIALIZER_H_