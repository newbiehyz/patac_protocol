//
#ifndef AUTODRIVE_MODEL_SERIALIZER_CAMERA_INTRINSICS_H_
#define AUTODRIVE_MODEL_SERIALIZER_CAMERA_INTRINSICS_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class CameraExtrinsicsMsg;
}
}

class CameraExtrinsicsData;

class CameraExtrinsicsSerializer : public message::ProtobufSerializer<workflow::proto::CameraExtrinsicsMsg, CameraExtrinsicsData> {
public:
  CameraExtrinsicsSerializer() noexcept = default;
  ~CameraExtrinsicsSerializer() noexcept = default;
  CameraExtrinsicsSerializer(CameraExtrinsicsSerializer const&) = default;
  CameraExtrinsicsSerializer& operator=(CameraExtrinsicsSerializer const&) = default;
  CameraExtrinsicsSerializer(CameraExtrinsicsSerializer&&) = default;
  CameraExtrinsicsSerializer& operator=(CameraExtrinsicsSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif