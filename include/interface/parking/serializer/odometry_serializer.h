//
#ifndef AUTODRIVE_INTERFACE_ODOMETRY_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_ODOMETRY_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class OdometryMsg;
}  // namespace proto
}  // namespace workflow

class OdometryData;

class OdometrySerializer
    : public message::ProtobufSerializer<workflow::proto::OdometryMsg,
                                         OdometryData> {
 public:
  OdometrySerializer() noexcept = default;
  ~OdometrySerializer() noexcept = default;
  OdometrySerializer(OdometrySerializer const&) = default;
  OdometrySerializer& operator=(OdometrySerializer const&) = default;
  OdometrySerializer(OdometrySerializer&&) = default;
  OdometrySerializer& operator=(OdometrySerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_ODOMETRY_SERIALIZER_H_