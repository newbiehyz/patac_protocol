//
#ifndef AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_GT_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_GT_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class DynamicObstaclesMsg;
}  // namespace proto
}  // namespace workflow

class DynamicObstaclesGtData;

class DynamicObstaclesGtSerializer
    : public message::ProtobufSerializer<workflow::proto::DynamicObstaclesMsg,
                                         DynamicObstaclesGtData> {
 public:
  DynamicObstaclesGtSerializer() noexcept = default;
  ~DynamicObstaclesGtSerializer() noexcept = default;
  DynamicObstaclesGtSerializer(DynamicObstaclesGtSerializer const&) = default;
  DynamicObstaclesGtSerializer& operator=(DynamicObstaclesGtSerializer const&) =
      default;
  DynamicObstaclesGtSerializer(DynamicObstaclesGtSerializer&&) = default;
  DynamicObstaclesGtSerializer& operator=(DynamicObstaclesGtSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_GT_SERIALIZER_H_