//
#include "interface/parking/fail_safe_result_data.h"

#include <cstring>

namespace autodrive {

FailsafeResultData::FailsafeResultData() noexcept
    : CommonMsgData{}, fail_safe_result_data_{} {}

FailsafeResultData::FailsafeResultData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, fail_safe_result_data_{} {}

FailsafeResultData::FailSafeResults const& FailsafeResultData::GetFailSafeResults()
    const noexcept {
  return fail_safe_result_data_;
}

FailsafeResultData::FailSafeResults&
FailsafeResultData::MutableFailSafeResults() noexcept {
  return fail_safe_result_data_;
}

void FailsafeResultData::SetFailSafeResults(
    FailSafeResults const& fail_safe_result_data) noexcept {
  fail_safe_result_data_ = fail_safe_result_data;
}

void FailsafeResultData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&fail_safe_result_data_, 0, sizeof(fail_safe_result_data_));
}

}  // namespace autodrive