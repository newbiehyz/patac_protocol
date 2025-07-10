//

#ifndef AUTODRIVE_INTERFACE_MODULE_TASK_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_MODULE_TASK_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ModuleTaskMsg;
}
}

class ModuleTaskData;

class ModuleTaskSerializer : public message::ProtobufSerializer<workflow::proto::ModuleTaskMsg, ModuleTaskData> {
public:
  ModuleTaskSerializer() noexcept = default;
  ~ModuleTaskSerializer() noexcept = default;
  ModuleTaskSerializer(ModuleTaskSerializer const&) = default;
  ModuleTaskSerializer& operator=(ModuleTaskSerializer const&) = default;
  ModuleTaskSerializer(ModuleTaskSerializer&&) = default;
  ModuleTaskSerializer& operator=(ModuleTaskSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif