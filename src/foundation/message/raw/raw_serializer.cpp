//
#include "foundation/message/raw/raw_serializer.h"

#include <iostream>

namespace autodrive {
namespace message {

int RawSerializer::Serialize(const SerializeMsg* data, DeserializeMsg* msg) const {
  // do nothing for raw serializer
  return 0;
}

int RawSerializer::Deserialize(const DeserializeMsg* msg, SerializeMsg* data) const {
  // do nothing for raw deserializer
  return 0;
}

}
}