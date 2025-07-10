//
#ifndef AUTODRIVE_INTERFACE_TRANSACTION_DATA_H_
#define AUTODRIVE_INTERFACE_TRANSACTION_DATA_H_

#include <array>
#include <cstring>
#include <foundation/type/common_msg_data.h>

namespace autodrive {

class TransactionData final : public CommonMsgData {
 public:
  static constexpr std::uint32_t kMaxTopicSize{64U};

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit TransactionData(std::int64_t timestamp) noexcept;

  TransactionData() noexcept;
  ~TransactionData() noexcept final = default;
  TransactionData(TransactionData const&) = default;
  TransactionData& operator=(TransactionData const&) = default;
  TransactionData(TransactionData&&) = default;
  TransactionData& operator=(TransactionData&&) = default;

  /// @brief Set topic
  ///
  /// @param topic
  /// @throw OutOfResourceException when topic is too long
  void SetTopic(std::string const& topic) noexcept(false);

  /// @brief Get topic
  ///
  /// @return topic
  std::string const GetTopic() noexcept;

  /// @brief Set frame id
  ///
  /// @param frame_id
  void SetDataFrameId(std::uint64_t frame_id) noexcept;

  /// @brief Get frame id
  ///
  /// @return frame id
  std::uint64_t const& GetDataFrameId() noexcept;

  /// @brief Set dag id
  ///
  /// @param dag_id
  void SetDagId(std::uint32_t dag_id) noexcept;

  /// @brief Get dag id
  ///
  /// @return dag id
  std::uint32_t const& GetDagId() noexcept;

  /// @brief Set enter time
  ///
  /// @param enter_time
  void SetEnterTime(std::uint64_t enter_time) noexcept;

  /// @brief Get enter time
  ///
  /// @return enter time
  std::uint64_t const& GetEnterTime() noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  /// @brief topic size
  std::uint8_t topic_size_;
  /// @brief topic
  std::array<char, kMaxTopicSize> topic_;
  /// @brief frame id
  std::uint64_t frame_id_;
  /// @brief dag id
  std::uint32_t dag_id_;
  /// @brief enter time
  std::uint64_t enter_time_;
};

}



#endif
