//
#ifndef AUTODRIVE_INTERFACE_PARSING_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_PARSING_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ParsingMsg;
}  // namespace proto
}  // namespace workflow

class ParsingData;

class ParsingSerializer
    : public message::ProtobufSerializer<workflow::proto::ParsingMsg,
                                         ParsingData> {
 public:
  ParsingSerializer() noexcept = default;
  ~ParsingSerializer() noexcept = default;
  ParsingSerializer(ParsingSerializer const&) = default;
  ParsingSerializer& operator=(ParsingSerializer const&) = default;
  ParsingSerializer(ParsingSerializer&&) = default;
  ParsingSerializer& operator=(ParsingSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_PARSING_SERIALIZER_H_