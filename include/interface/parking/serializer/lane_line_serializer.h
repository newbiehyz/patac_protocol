//
#ifndef AUTODRIVE_INTERFACE_LANE_LINE_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_LANE_LINE_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class LaneLinePointsMsg;
}  // namespace proto
}  // namespace workflow

class LaneLineData;

class LaneLineSerializer
    : public message::ProtobufSerializer<workflow::proto::LaneLinePointsMsg,
                                         LaneLineData> {
 public:
  LaneLineSerializer() noexcept = default;
  ~LaneLineSerializer() noexcept = default;
  LaneLineSerializer(LaneLineSerializer const&) = default;
  LaneLineSerializer& operator=(LaneLineSerializer const&) = default;
  LaneLineSerializer(LaneLineSerializer&&) = default;
  LaneLineSerializer& operator=(LaneLineSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_LANE_LINE_SERIALIZER_H_