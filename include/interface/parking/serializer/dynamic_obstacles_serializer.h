//
#ifndef AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class DynamicObstaclesMsg;
}  // namespace proto
}  // namespace workflow

class DynamicObstaclesData;

class DynamicObstaclesSerializer
    : public message::ProtobufSerializer<workflow::proto::DynamicObstaclesMsg,
                                         DynamicObstaclesData> {
 public:
  DynamicObstaclesSerializer() noexcept = default;
  ~DynamicObstaclesSerializer() noexcept = default;
  DynamicObstaclesSerializer(DynamicObstaclesSerializer const&) = default;
  DynamicObstaclesSerializer& operator=(DynamicObstaclesSerializer const&) =
      default;
  DynamicObstaclesSerializer(DynamicObstaclesSerializer&&) = default;
  DynamicObstaclesSerializer& operator=(DynamicObstaclesSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_SERIALIZER_H_