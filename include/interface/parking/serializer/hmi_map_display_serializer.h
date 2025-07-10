//
#ifndef AUTODRIVE_INTERFACE_HMI_MAP_DISPLAY_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_HMI_MAP_DISPLAY_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class HmiMapDisplayMsg;
} // namespace proto
} // namespace workflow

class HmiMapDisplayData;

class HmiMapDisplaySerializer
    : public message::ProtobufSerializer<workflow::proto::HmiMapDisplayMsg,
                                         HmiMapDisplayData> {
public:
  HmiMapDisplaySerializer() noexcept = default;
  ~HmiMapDisplaySerializer() noexcept = default;
  HmiMapDisplaySerializer(HmiMapDisplaySerializer const &) = default;
  HmiMapDisplaySerializer &operator=(HmiMapDisplaySerializer const &) = default;
  HmiMapDisplaySerializer(HmiMapDisplaySerializer &&) = default;
  HmiMapDisplaySerializer &operator=(HmiMapDisplaySerializer &&) = default;

  int Serialize(const message::SerializeMsg *data_ptr,
                message::DeserializeMsg *msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg *msg_ptr,
                  message::SerializeMsg *data_ptr) const override;
};

} // namespace autodrive

#endif // AUTODRIVE_INTERFACE_HMI_MAP_DISPLAY_SERIALIZER_H_