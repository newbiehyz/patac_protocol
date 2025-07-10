//
#ifndef AUTODRIVE_MODEL_SERIALIZER_CAMERA_INTRINSICS_H_
#define AUTODRIVE_MODEL_SERIALIZER_CAMERA_INTRINSICS_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class CameraIntrinsicsMsg;
}
}

class CameraIntrinsicsData;

class CameraIntrinsicsSerializer : public message::ProtobufSerializer<workflow::proto::CameraIntrinsicsMsg, CameraIntrinsicsData> {
public:
  CameraIntrinsicsSerializer() noexcept = default;
  ~CameraIntrinsicsSerializer() noexcept = default;
  CameraIntrinsicsSerializer(CameraIntrinsicsSerializer const&) = default;
  CameraIntrinsicsSerializer& operator=(CameraIntrinsicsSerializer const&) = default;
  CameraIntrinsicsSerializer(CameraIntrinsicsSerializer&&) = default;
  CameraIntrinsicsSerializer& operator=(CameraIntrinsicsSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif