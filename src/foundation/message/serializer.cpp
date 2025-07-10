//
#include "foundation/message/serializer.h"

namespace autodrive {
namespace message {

int DefaultSerializer::Serialize(const SerializeMsg* data_ptr,
    DeserializeMsg* msg) const {
  return 0;
}

int DefaultSerializer::Deserialize(const DeserializeMsg* msg_ptr,
    SerializeMsg* data_ptr) const {
  return 0;
}

}
}