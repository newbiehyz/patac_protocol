//
#ifndef AUTODRIVE_INTERFACE_HMI_MAP_PARKING_SLOTS_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_HMI_MAP_PARKING_SLOTS_SERIALIZER_H_
#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class MapParkingSlotsMsg;
} // namespace proto
} // namespace workflow

class HmiMapParkingSlotsData;
class HmiMapParkingSlotsSerializer
    : public message::ProtobufSerializer<workflow::proto::MapParkingSlotsMsg,
                                         HmiMapParkingSlotsData> {
public:
  HmiMapParkingSlotsSerializer() noexcept = default;
  ~HmiMapParkingSlotsSerializer() noexcept = default;
  HmiMapParkingSlotsSerializer(HmiMapParkingSlotsSerializer const &) = default;
  HmiMapParkingSlotsSerializer &operator=(HmiMapParkingSlotsSerializer const &) = default;
  HmiMapParkingSlotsSerializer(HmiMapParkingSlotsSerializer &&) = default;
  HmiMapParkingSlotsSerializer &operator=(HmiMapParkingSlotsSerializer &&) = default;

  int Serialize(const message::SerializeMsg *data_ptr,
                message::DeserializeMsg *msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg *msg_ptr,
                  message::SerializeMsg *data_ptr) const override;
};
}
#endif //AUTODRIVE_INTERFACE_HMI_MAP_PARKING_SLOTS_SERIALIZER_H_