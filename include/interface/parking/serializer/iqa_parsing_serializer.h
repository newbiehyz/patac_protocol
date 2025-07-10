//
#ifndef AUTODRIVE_INTERFACE_IQA_PARSING_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_IQA_PARSING_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class IqaParsingMsg;
}  // namespace proto
}  // namespace workflow

class IqaParsingData;

class IqaParsingSerializer
    : public message::ProtobufSerializer<workflow::proto::IqaParsingMsg,
                                         IqaParsingData> {
 public:
  IqaParsingSerializer() noexcept = default;
  ~IqaParsingSerializer() noexcept = default;
  IqaParsingSerializer(IqaParsingSerializer const&) = default;
  IqaParsingSerializer& operator=(IqaParsingSerializer const&) = default;
  IqaParsingSerializer(IqaParsingSerializer&&) = default;
  IqaParsingSerializer& operator=(IqaParsingSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_IPM_PARSING_SERIALIZER_H_