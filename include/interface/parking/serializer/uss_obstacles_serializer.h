//
#ifndef AUTODRIVE_INTERFACE_USS_OBSTACLES_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_USS_OBSTACLES_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class UssObstaclesMsg;
}  // namespace proto
}  // namespace workflow

class UssObstaclesData;

class UssObstaclesDataSerializer
    : public message::ProtobufSerializer<workflow::proto::UssObstaclesMsg,
                                         UssObstaclesData> {
 public:
  UssObstaclesDataSerializer() noexcept = default;
  ~UssObstaclesDataSerializer() noexcept = default;
  UssObstaclesDataSerializer(UssObstaclesDataSerializer const&) = default;
  UssObstaclesDataSerializer& operator=(UssObstaclesDataSerializer const&) =
      default;
  UssObstaclesDataSerializer(UssObstaclesDataSerializer&&) = default;
  UssObstaclesDataSerializer& operator=(UssObstaclesDataSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_USS_OBSTACLES_SERIALIZER_H_