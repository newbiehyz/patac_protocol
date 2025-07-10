//
#ifndef AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DEBUG_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DEBUG_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
    class VisionDebugMsg;
}  // namespace proto
}  // namespace workflow

class VisionDebugData;

class VisionDebugSerializer
    : public message::ProtobufSerializer<workflow::proto::VisionDebugMsg,
                                         VisionDebugData>
{
public:
    VisionDebugSerializer() noexcept = default;
    ~VisionDebugSerializer() noexcept = default;
    VisionDebugSerializer(VisionDebugSerializer const &) = default;
    VisionDebugSerializer &operator=(VisionDebugSerializer const &) =
        default;
    VisionDebugSerializer(VisionDebugSerializer &&) = default;
    VisionDebugSerializer &operator=(VisionDebugSerializer &&) = default;

    int Serialize(const message::SerializeMsg *data_ptr,
                  message::DeserializeMsg *msg_ptr) const override;
    int Deserialize(const message::DeserializeMsg *msg_ptr,
                    message::SerializeMsg *data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_DYNAMIC_OBSTACLES_DEBUG_SERIALIZER_H_