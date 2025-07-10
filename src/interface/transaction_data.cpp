//

#include "interface/transaction_data.h"
#include <utils/exception.h>

namespace autodrive {

TransactionData::TransactionData() noexcept : CommonMsgData{},
    topic_size_{0U}, topic_{}, frame_id_{0U}, dag_id_{}, enter_time_{0U} {
}

TransactionData::TransactionData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp}, topic_size_{0U}, topic_{}, frame_id_{0U}, dag_id_{}, enter_time_{0U} {
}

void TransactionData::SetTopic(std::string const& topic) noexcept(false) {
  if (topic.size() > kMaxTopicSize) {
    throw OutOfResourceException("topic is too long");
  }
  topic_size_ = topic.size();
  memcpy(topic_.data(), topic.c_str(), topic.size());
}

std::string const TransactionData::GetTopic() noexcept {
  std::string topic;
  topic.assign(topic_.begin(), topic_.begin() + topic_size_);
  return topic;
}

void TransactionData::SetDataFrameId(std::uint64_t frame_id) noexcept {
  frame_id_ = frame_id;
}

std::uint64_t const& TransactionData::GetDataFrameId() noexcept {
  return frame_id_;
}

void TransactionData::SetDagId(std::uint32_t dag_id) noexcept {
  dag_id_ = dag_id;
}

std::uint32_t const& TransactionData::GetDagId() noexcept {
  return dag_id_;
}

void TransactionData::SetEnterTime(std::uint64_t enter_time) noexcept {
  enter_time_ = enter_time;
}

std::uint64_t const& TransactionData::GetEnterTime() noexcept {
  return enter_time_;
}

void TransactionData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&topic_, 0, sizeof(topic_));
  topic_size_ = 0U;
  frame_id_ = 0U;
  dag_id_ = 0U;
  enter_time_ = 0U;
}

}