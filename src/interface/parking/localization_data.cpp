//
#include "interface/parking/localization_data.h"

#include <cstring>

namespace autodrive {

LocalizationData::LocalizationData() noexcept
    : CommonMsgData{}, localization_data_{} {}

LocalizationData::LocalizationData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, localization_data_{} {}

LocalizationData::Localization const& LocalizationData::GetLocalization()
    const noexcept {
  return localization_data_;
}

LocalizationData::Localization&
LocalizationData::MutableLocalization() noexcept {
  return localization_data_;
}

void LocalizationData::SetLocalization(
    Localization const& localization_data) noexcept {
  localization_data_ = localization_data;
}

void LocalizationData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&localization_data_, 0, sizeof(localization_data_));
}

}  // namespace autodrive