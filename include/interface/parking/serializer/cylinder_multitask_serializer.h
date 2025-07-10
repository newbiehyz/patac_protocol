//
#ifndef AUTODRIVE_INTERFACE_CYLINDER_MULTITASK_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_CYLINDER_MULTITASK_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class CylinderMultiTask;
}  // namespace proto
}  // namespace workflow

class CylinderMultiTaskData;

class CylinderMultiTaskSerializer
    : public message::ProtobufSerializer<workflow::proto::CylinderMultiTask,
                                         CylinderMultiTaskData> {
 public:
  CylinderMultiTaskSerializer() noexcept = default;
  ~CylinderMultiTaskSerializer() noexcept = default;
  CylinderMultiTaskSerializer(CylinderMultiTaskSerializer const&) = default;
  CylinderMultiTaskSerializer& operator=(CylinderMultiTaskSerializer const&) = default;
  CylinderMultiTaskSerializer(CylinderMultiTaskSerializer&&) = default;
  CylinderMultiTaskSerializer& operator=(CylinderMultiTaskSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr,
                message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr,
                  message::SerializeMsg* data_ptr) const override;
};

}  // namespace autodrive

#endif  // AUTODRIVE_INTERFACE_CYLINDER_MULTITASK_SERIALIZER_H_