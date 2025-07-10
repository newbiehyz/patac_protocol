//
#ifndef AUTODRIVE_INTERFACE_LANE_MULTITASK_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_LANE_MULTITASK_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class LaneMultitaskMsg;
}  // namespace proto
}  // namespace workflow

class LaneMultitaskData;

class LaneMultitaskSerializer
    : public message::ProtobufSerializer<workflow::proto::LaneMultitaskMsg,
                                         LaneMultitaskData> {
 public:
  LaneMultitaskSerializer() noexcept = default;
  ~LaneMultitaskSerializer() noexcept = default;
  LaneMultitaskSerializer(LaneMultitaskSerializer const&) = default;
  LaneMultitaskSerializer& operator=(LaneMultitaskSerializer const&) = default;
  LaneMultitaskSerializer(LaneMultitaskSerializer&&) = default;
  LaneMultitaskSerializer& operator=(LaneMultitaskSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_LANE_MULTITASK_SERIALIZER_H_