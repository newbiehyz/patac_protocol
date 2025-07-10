//
#ifndef AUTODRIVE_INTERFACE_SYNC_CYLINDER_PARSING_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_SYNC_CYLINDER_PARSING_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class SyncCylinderParsingMsg;
}  // namespace proto
}  // namespace workflow

class SyncCylinderParsingData;

class SyncCylinderParsingSerializer
    : public message::ProtobufSerializer<
          workflow::proto::SyncCylinderParsingMsg, SyncCylinderParsingData> {
 public:
  SyncCylinderParsingSerializer() noexcept = default;
  ~SyncCylinderParsingSerializer() noexcept = default;
  SyncCylinderParsingSerializer(SyncCylinderParsingSerializer const&) = default;
  SyncCylinderParsingSerializer& operator=(
      SyncCylinderParsingSerializer const&) = default;
  SyncCylinderParsingSerializer(SyncCylinderParsingSerializer&&) = default;
  SyncCylinderParsingSerializer& operator=(SyncCylinderParsingSerializer&&) =
      default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_SYNC_CYLINDER_PARSING_SERIALIZER_H_