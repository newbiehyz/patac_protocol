//
#include "model/eol_initial_data.h"

#include <cstring>

namespace autodrive {

EolInitialData::EolInitialData() noexcept
    : CommonMsgData{}, initial_data_{0U} {}

EolInitialData::EolInitialData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, initial_data_{0U} {}

EolInitialData::EolInitial const& EolInitialData::GetInitialData()
    const noexcept {
  return initial_data_;
}

EolInitialData::EolInitial& EolInitialData::MutableInitialData() noexcept {
  return initial_data_;
}

void EolInitialData::SetInitialData(const EolInitial& initial_data) noexcept {
  initial_data_ = initial_data;
}

void EolInitialData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&initial_data_, 0, sizeof(initial_data_));
}

}  // namespace autodrive