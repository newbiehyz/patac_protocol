//
#ifndef AUTODRIVE_INTERFACE_Hmi_RENDER_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_Hmi_RENDER_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class HmiRenderMsg;
}  // namespace proto
}  // namespace workflow

class HmiRenderData;

class HmiRenderSerializer
    : public message::ProtobufSerializer<workflow::proto::HmiRenderMsg,
                                         HmiRenderData> {
 public:
  HmiRenderSerializer() noexcept = default;
  ~HmiRenderSerializer() noexcept = default;
  HmiRenderSerializer(HmiRenderSerializer const&) = default;
  HmiRenderSerializer& operator=(HmiRenderSerializer const&) = default;
  HmiRenderSerializer(HmiRenderSerializer&&) = default;
  HmiRenderSerializer& operator=(HmiRenderSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_Hmi_RENDER_SERIALIZER_H_