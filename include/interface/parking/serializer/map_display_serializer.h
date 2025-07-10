//
#ifndef AUTODRIVE_INTERFACE_MAP_DISPLAY_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_MAP_DISPLAY_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class MapDisplayMsg;
}  // namespace proto
}  // namespace workflow

class MapDisplayData;

class MapDisplaSerializer
    : public message::ProtobufSerializer<workflow::proto::MapDisplayMsg,
                                         MapDisplayData> {
 public:
  MapDisplaSerializer() noexcept = default;
  ~MapDisplaSerializer() noexcept = default;
  MapDisplaSerializer(MapDisplaSerializer const&) = default;
  MapDisplaSerializer& operator=(MapDisplaSerializer const&) = default;
  MapDisplaSerializer(MapDisplaSerializer&&) = default;
  MapDisplaSerializer& operator=(MapDisplaSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_MAP_DISPLAY_SERIALIZER_H_