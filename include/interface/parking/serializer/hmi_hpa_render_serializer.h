//
#ifndef AUTODRIVE_INTERFACE_HMI_HPA_RENDER_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_HMI_HPA_RENDER_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class HmiHpaRenderMsg;
} // namespace proto
} // namespace workflow

class HmiHpaRenderData;

class HmiHpaRenderSerializer
    : public message::ProtobufSerializer<workflow::proto::HmiHpaRenderMsg,
                                         HmiHpaRenderData> {
public:
  HmiHpaRenderSerializer() noexcept = default;
  ~HmiHpaRenderSerializer() noexcept = default;
  HmiHpaRenderSerializer(HmiHpaRenderSerializer const &) = default;
  HmiHpaRenderSerializer &operator=(HmiHpaRenderSerializer const &) = default;
  HmiHpaRenderSerializer(HmiHpaRenderSerializer &&) = default;
  HmiHpaRenderSerializer &operator=(HmiHpaRenderSerializer &&) = default;

  int Serialize(const message::SerializeMsg *data_ptr,
                message::DeserializeMsg *msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg *msg_ptr,
                  message::SerializeMsg *data_ptr) const override;
};

} // namespace autodrive

#endif // AUTODRIVE_INTERFACMI_HPAHmi_RENDER_SERIALIZER_H_