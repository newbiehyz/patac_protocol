//
#ifndef AUTODRIVE_INTERFACE_FAIL_SAFE_RESULT_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_FAIL_SAFE_RESULT_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class FailsafeResultMsg;
}  // namespace proto
}  // namespace workflow

class FailsafeResultData;

class FailsafeResultSerializer
    : public message::ProtobufSerializer<workflow::proto::FailsafeResultMsg,
                                         FailsafeResultData> {
 public:
  FailsafeResultSerializer() noexcept = default;
  ~FailsafeResultSerializer() noexcept = default;
  FailsafeResultSerializer(FailsafeResultSerializer const&) = default;
  FailsafeResultSerializer& operator=(FailsafeResultSerializer const&) =
      default;
  FailsafeResultSerializer(FailsafeResultSerializer&&) = default;
  FailsafeResultSerializer& operator=(FailsafeResultSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_FAIL_SAFE_RESULT_SERIALIZER_H_