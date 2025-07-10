//
#ifndef AUTODRIVE_INTERFACE_HMI_HPA_RENDER_ALL_MAP_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_HMI_HPA_RENDER_ALL_MAP_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class HmiHpaRenderAllMapMsg;
} // namespace proto
} // namespace workflow

class HmiHpaRenderAllMapData;

class HmiHpaRenderAllMapSerializer
    : public message::ProtobufSerializer<workflow::proto::HmiHpaRenderAllMapMsg,
                                         HmiHpaRenderAllMapData> {
public:
  HmiHpaRenderAllMapSerializer() noexcept = default;
  ~HmiHpaRenderAllMapSerializer() noexcept = default;
  HmiHpaRenderAllMapSerializer(HmiHpaRenderAllMapSerializer const &) = default;
  HmiHpaRenderAllMapSerializer &operator=(HmiHpaRenderAllMapSerializer const &) = default;
  HmiHpaRenderAllMapSerializer(HmiHpaRenderAllMapSerializer &&) = default;
  HmiHpaRenderAllMapSerializer &operator=(HmiHpaRenderAllMapSerializer &&) = default;

  int Serialize(const message::SerializeMsg *data_ptr,
                message::DeserializeMsg *msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg *msg_ptr,
                  message::SerializeMsg *data_ptr) const override;
};

} // namespace autodrive

#endif // AUTODRIVE_INTERFACE_HMI_HPA_RENDER_ALL_MAP_SERIALIZER_H_