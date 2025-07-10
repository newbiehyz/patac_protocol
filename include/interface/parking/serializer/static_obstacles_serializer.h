//
#ifndef AUTODRIVE_INTERFACE_STATIC_OBSTACLES_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_STATIC_OBSTACLES_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class StaticObstaclesMsg;
}  // namespace proto
}  // namespace workflow

class StaticObstaclesData;

class StaticObstaclesSerializer
    : public message::ProtobufSerializer<workflow::proto::StaticObstaclesMsg,
                                         StaticObstaclesData> {
 public:
  StaticObstaclesSerializer() noexcept = default;
  ~StaticObstaclesSerializer() noexcept = default;
  StaticObstaclesSerializer(StaticObstaclesSerializer const&) = default;
  StaticObstaclesSerializer& operator=(StaticObstaclesSerializer const&) =
      default;
  StaticObstaclesSerializer(StaticObstaclesSerializer&&) = default;
  StaticObstaclesSerializer& operator=(StaticObstaclesSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_STATIC_OBSTACLES_SERIALIZER_H_