//
#include "foundation/message/flex_ray.h"

namespace autodrive {
namespace message {

int DefaultFlexRay::Mapping(const ProvideMsg* msg_ptr,
    MappingMsg* data_ptr) const {
  return 0;
}

}
}