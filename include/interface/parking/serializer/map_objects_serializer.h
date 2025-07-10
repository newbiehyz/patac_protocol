//
#ifndef AUTODRIVE_INTERFACE_MAP_OBJECTS_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_MAP_OBJECTS_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class MapObjectsMsg;
}  // namespace proto
}  // namespace workflow

class MapObjectsData;

class MapObjectsSerializer
    : public message::ProtobufSerializer<workflow::proto::MapObjectsMsg,
                                         MapObjectsData> {
 public:
  MapObjectsSerializer() noexcept = default;
  ~MapObjectsSerializer() noexcept = default;
  MapObjectsSerializer(MapObjectsSerializer const&) = default;
  MapObjectsSerializer& operator=(MapObjectsSerializer const&) = default;
  MapObjectsSerializer(MapObjectsSerializer&&) = default;
  MapObjectsSerializer& operator=(MapObjectsSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_MAP_OBJECTS_SERIALIZER_H_