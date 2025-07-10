//

#ifndef AUTODRIVE_INTERFACE_TRANSACTION_SERIALIZER_H_
#define AUTODRIVE_INTERFACE_TRANSACTION_SERIALIZER_H_

#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class TransactionMsg;
}
}

class TransactionData;

class TransactionSerializer : public message::ProtobufSerializer<workflow::proto::TransactionMsg, TransactionData> {
public:
  TransactionSerializer() noexcept = default;
  ~TransactionSerializer() noexcept = default;
  TransactionSerializer(TransactionSerializer const&) = default;
  TransactionSerializer& operator=(TransactionSerializer const&) = default;
  TransactionSerializer(TransactionSerializer&&) = default;
  TransactionSerializer& operator=(TransactionSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;
};

}

#endif