//
#ifndef AUTODRIVE_MESSAGE_PROTO_SERIALIZER_H
#define AUTODRIVE_MESSAGE_PROTO_SERIALIZER_H

#include "foundation/message/serializer.h"

namespace autodrive {
namespace message {

template <typename ProtobufMsg, typename DataMsg>
class ProtobufSerializer : public SerializerPolicy {
 public:
  using DMsgType = DeserializeDataMsg<ProtobufMsg>;
  using SMsgType = SerializeDataMsg<DataMsg>;
  int Serialize(const SerializeMsg* data, DeserializeMsg* msg) const override;
  int Deserialize(const DeserializeMsg* msg, SerializeMsg* data) const override;
};

template <typename ProtobufMsg, typename DataMsg>
int ProtobufSerializer<ProtobufMsg, DataMsg>::Serialize(
    const SerializeMsg* data, DeserializeMsg* msg) const {
  return 0;
}

template <typename ProtobufMsg, typename DataMsg>
int ProtobufSerializer<ProtobufMsg, DataMsg>::Deserialize(const DeserializeMsg* msg,
    SerializeMsg* data) const {
  return 0;
}

}
}

#endif