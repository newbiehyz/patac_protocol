//
#ifndef AUTODRIVE_INTERFACE_IPM_PARSING_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_IPM_PARSING_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class IpmParsingMsg;
}  // namespace proto
}  // namespace workflow

class IpmParsingData;

class IpmParsingSerializer
    : public message::ProtobufSerializer<workflow::proto::IpmParsingMsg,
                                         IpmParsingData> {
 public:
  IpmParsingSerializer() noexcept = default;
  ~IpmParsingSerializer() noexcept = default;
  IpmParsingSerializer(IpmParsingSerializer const&) = default;
  IpmParsingSerializer& operator=(IpmParsingSerializer const&) = default;
  IpmParsingSerializer(IpmParsingSerializer&&) = default;
  IpmParsingSerializer& operator=(IpmParsingSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_IPM_PARSING_SERIALIZER_H_