//
#ifndef AUTODRIVE_INTERFACE_REFERENCE_LINES_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_REFERENCE_LINES_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ReferenceLinesMsg;
}  // namespace proto
}  // namespace workflow

class ReferenceLinesData;

class ReferenceLinesSerializer
    : public message::ProtobufSerializer<workflow::proto::ReferenceLinesMsg,
                                         ReferenceLinesData> {
 public:
  ReferenceLinesSerializer() noexcept = default;
  ~ReferenceLinesSerializer() noexcept = default;
  ReferenceLinesSerializer(ReferenceLinesSerializer const&) = default;
  ReferenceLinesSerializer& operator=(ReferenceLinesSerializer const&) =
      default;
  ReferenceLinesSerializer(ReferenceLinesSerializer&&) = default;
  ReferenceLinesSerializer& operator=(ReferenceLinesSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_REFERENCE_LINES_SERIALIZER_H_