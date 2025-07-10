//
#include "foundation/message/message.h"
#include "foundation/message/data_ref.h"

namespace autodrive {
namespace message {

Message::Message() noexcept : data_{nullptr} {}

DataRefSp Message::GetData() const noexcept {
  return data_;
}

void Message::SetData(DataRefSp const& data) noexcept {
  data_ = std::move(data);
}

}
}