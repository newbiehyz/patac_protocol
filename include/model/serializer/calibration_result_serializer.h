//
#ifndef AUTODRIVE_MODEL_SERIALIZER_CALIBRATION_RESULT_H_
#define AUTODRIVE_MODEL_SERIALIZER_CALIBRATION_RESULT_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class CalibrationResultMsg;
}
}

class CalibrationResultData;

class CalibrationResultSerializer : public message::ProtobufSerializer<workflow::proto::CalibrationResultMsg, CalibrationResultData> {
public:
  CalibrationResultSerializer() noexcept = default;
  ~CalibrationResultSerializer() noexcept = default;
  CalibrationResultSerializer(CalibrationResultSerializer const&) = default;
  CalibrationResultSerializer& operator=(CalibrationResultSerializer const&) = default;
  CalibrationResultSerializer(CalibrationResultSerializer&&) = default;
  CalibrationResultSerializer& operator=(CalibrationResultSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif