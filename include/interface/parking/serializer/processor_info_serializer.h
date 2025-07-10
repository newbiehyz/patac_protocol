//
#ifndef AUTODRIVE_INTERFACE_PROCESSOR_INFO_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_PROCESSOR_INFO_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ProcessorInfoMsg;
}  // namespace proto
}  // namespace workflow

class ProcessorInfoData;

class ProcessorInfoSerializer
    : public message::ProtobufSerializer<workflow::proto::ProcessorInfoMsg,
                                         ProcessorInfoData> {
 public:
  ProcessorInfoSerializer() noexcept = default;
  ~ProcessorInfoSerializer() noexcept = default;
  ProcessorInfoSerializer(ProcessorInfoSerializer const&) = default;
  ProcessorInfoSerializer& operator=(ProcessorInfoSerializer const&) = default;
  ProcessorInfoSerializer(ProcessorInfoSerializer&&) = default;
  ProcessorInfoSerializer& operator=(ProcessorInfoSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_PROCESSOR_INFO_SERIALIZER_H_