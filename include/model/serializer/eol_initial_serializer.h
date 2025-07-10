//
#ifndef AUTODRIVE_MODEL_SERIALIZER_EOL_INITIAL_H_
#define AUTODRIVE_MODEL_SERIALIZER_EOL_INITIAL_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class EolInitialMsg;
}
}

class EolInitialData;

class EolInitialSerializer : public message::ProtobufSerializer<workflow::proto::EolInitialMsg, EolInitialData> {
public:
  EolInitialSerializer() noexcept = default;
  ~EolInitialSerializer() noexcept = default;
  EolInitialSerializer(EolInitialSerializer const&) = default;
  EolInitialSerializer& operator=(EolInitialSerializer const&) = default;
  EolInitialSerializer(EolInitialSerializer&&) = default;
  EolInitialSerializer& operator=(EolInitialSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif