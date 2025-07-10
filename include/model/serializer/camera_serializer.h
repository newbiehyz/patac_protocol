//
#ifndef AUTODRIVE_MODEL_SERIALIZER_CAMERA_H_
#define AUTODRIVE_MODEL_SERIALIZER_CAMERA_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class CameraMsg;
}
}

class CameraData;

class CameraSerializer : public message::ProtobufSerializer<workflow::proto::CameraMsg, CameraData> {
public:
  CameraSerializer() noexcept = default;
  ~CameraSerializer() noexcept = default;
  CameraSerializer(CameraSerializer const&) = default;
  CameraSerializer& operator=(CameraSerializer const&) = default;
  CameraSerializer(CameraSerializer&&) = default;
  CameraSerializer& operator=(CameraSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif