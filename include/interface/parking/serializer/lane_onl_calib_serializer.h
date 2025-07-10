//
#ifndef AUTODRIVE_INTERFACE_PARKING_SERIALIZERLANE_ONL_CALIB_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_PARKING_SERIALIZERLANE_ONL_CALIB_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class LaneOnlCalibMsg;
}  // namespace proto
}  // namespace workflow

class LaneOnlCalibData;

class LaneOnlCalibSerializer
    : public message::ProtobufSerializer<workflow::proto::LaneOnlCalibMsg,
                                         LaneOnlCalibData> {
 public:
  LaneOnlCalibSerializer() noexcept = default;
  ~LaneOnlCalibSerializer() noexcept = default;
  LaneOnlCalibSerializer(LaneOnlCalibSerializer const&) = default;
  LaneOnlCalibSerializer& operator=(LaneOnlCalibSerializer const&) = default;
  LaneOnlCalibSerializer(LaneOnlCalibSerializer&&) = default;
  LaneOnlCalibSerializer& operator=(LaneOnlCalibSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_PARKING_SERIALIZERLANE_ONL_CALIB_SERIALIZER_H_