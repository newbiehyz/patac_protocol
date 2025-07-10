//
#include "foundation/message/data_ref.h"

namespace autodrive {
namespace message {

DataRef::DataRef() noexcept : ref_data_{nullptr, 0U} {}

DataRef::DataRef(RefData const& ref_data) noexcept :
    ref_data_{ref_data} {}

void DataRef::SetData(RefData const& ref_data) noexcept(false) {
  ref_data_ = ref_data;
}

DataRef::RefData DataRef::GetData() const noexcept(false) {
  return ref_data_;
}

}
}