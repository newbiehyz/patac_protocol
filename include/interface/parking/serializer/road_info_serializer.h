//
#ifndef AUTODRIVE_INTERFACE_ROAD_INFO_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_ROAD_INFO_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class RoadInfoMsg;
}  // namespace proto
}  // namespace workflow

class RoadInfoData;

class RoadInfoSerializer
    : public message::ProtobufSerializer<workflow::proto::RoadInfoMsg,
                                         RoadInfoData> {
 public:
  RoadInfoSerializer() noexcept = default;
  ~RoadInfoSerializer() noexcept = default;
  RoadInfoSerializer(RoadInfoSerializer const&) = default;
  RoadInfoSerializer& operator=(RoadInfoSerializer const&) = default;
  RoadInfoSerializer(RoadInfoSerializer&&) = default;
  RoadInfoSerializer& operator=(RoadInfoSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_ROAD_INFO_SERIALIZER_H_