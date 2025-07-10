//
#ifndef AUTODRIVE_INTERFACE_SPEED_BUMPS_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_SPEED_BUMPS_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class SpeedBumpsMsg;
}  // namespace proto
}  // namespace workflow

class SpeedBumpData;

class SpeedBumpSerializer
    : public message::ProtobufSerializer<workflow::proto::SpeedBumpsMsg,
                                         SpeedBumpData> {
 public:
  SpeedBumpSerializer() noexcept = default;
  ~SpeedBumpSerializer() noexcept = default;
  SpeedBumpSerializer(SpeedBumpSerializer const&) = default;
  SpeedBumpSerializer& operator=(SpeedBumpSerializer const&) = default;
  SpeedBumpSerializer(SpeedBumpSerializer&&) = default;
  SpeedBumpSerializer& operator=(SpeedBumpSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_SPEED_BUMPS_SERIALIZER_H_