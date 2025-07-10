//
#ifndef AUTODRIVE_MESSAGE_RAW_H
#define AUTODRIVE_MESSAGE_RAW_H

#include "foundation/message/serializer.h"

namespace autodrive {
namespace message {

class RawSerializer : public SerializerPolicy {
public:
  int Serialize(const SerializeMsg* data, DeserializeMsg* msg) const override;
  int Deserialize(const DeserializeMsg* msg, SerializeMsg* data) const override;
};

}
}

#endif