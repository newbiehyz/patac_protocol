//
#ifndef AUTODRIVE_INTERFACE_MAPLOC_SYSTEM_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_MAPLOC_SYSTEM_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class MapLocSystemMsg;
}  // namespace proto
}  // namespace workflow

class MapLocSystemData;

class MapLocSystemSerializer
    : public message::ProtobufSerializer<workflow::proto::MapLocSystemMsg,
                                         MapLocSystemData> {
 public:
  MapLocSystemSerializer() noexcept = default;
  ~MapLocSystemSerializer() noexcept = default;
  MapLocSystemSerializer(MapLocSystemSerializer const&) = default;
  MapLocSystemSerializer& operator=(MapLocSystemSerializer const&) = default;
  MapLocSystemSerializer(MapLocSystemSerializer&&) = default;
  MapLocSystemSerializer& operator=(MapLocSystemSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_MAPLOC_SYSTEM_SERIALIZER_H_