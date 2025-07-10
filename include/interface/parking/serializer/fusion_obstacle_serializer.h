//
#ifndef AUTODRIVE_INTERFACE_FUSION_OBSTACLE_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_FUSION_OBSTACLE_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class FusionObstacleMsg;
}  // namespace proto
}  // namespace workflow

class FusionObstacleData;

class FusionObstacleSerializer
    : public message::ProtobufSerializer<workflow::proto::FusionObstacleMsg,
                                         FusionObstacleData> {
 public:
  FusionObstacleSerializer() noexcept = default;
  ~FusionObstacleSerializer() noexcept = default;
  FusionObstacleSerializer(FusionObstacleSerializer const&) = default;
  FusionObstacleSerializer& operator=(FusionObstacleSerializer const&) = default;
  FusionObstacleSerializer(FusionObstacleSerializer&&) = default;
  FusionObstacleSerializer& operator=(FusionObstacleSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_FUSION_OBSTACLE_SERIALIZER_H_