//
#include "interface/parking/iqa_parsing_data.h"

#include <cstring>

namespace autodrive {

IqaParsingData::IqaParsingData() noexcept
    : CommonMsgData{}, iqa_parsing_data_{0} {}

IqaParsingData::IqaParsingData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, iqa_parsing_data_{0}, scene_parsing_data_{0} {}

IqaParsingData::IqaParsingResults const& IqaParsingData::GetIqaParsingResults() const noexcept {
  return iqa_parsing_data_;
}

IqaParsingData::SceneParsingResults const& IqaParsingData::GetSceneParsingResults() const noexcept {
  return scene_parsing_data_;
}

IqaParsingData::IqaParsingResults& IqaParsingData::MutableIqaParsingResults() noexcept {
  return iqa_parsing_data_;
}

IqaParsingData::SceneParsingResults& IqaParsingData::MutableSceneParsingResults() noexcept {
  return scene_parsing_data_;
}

void IqaParsingData::SetIqaParsingResults(
    IqaParsingResults const& iqa_parsing_data) noexcept {
  iqa_parsing_data_ = iqa_parsing_data;
}

void IqaParsingData::SetSceneParsingResults(
    SceneParsingResults const& scene_parsing_data) noexcept {
  scene_parsing_data_ = scene_parsing_data;
}

void IqaParsingData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&iqa_parsing_data_, 0, sizeof(iqa_parsing_data_));
  memset(&scene_parsing_data_, 0, sizeof(scene_parsing_data_));
}

}  // namespace autodrive