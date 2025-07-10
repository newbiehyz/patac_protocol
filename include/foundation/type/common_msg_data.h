//
#ifndef AUTODRIVE_TYPE_COMMON_MSG_DATA_H_
#define AUTODRIVE_TYPE_COMMON_MSG_DATA_H_

#include <cstdint>

#include "time_series_data.h"

namespace autodrive {

// Common msg data
class CommonMsgData : public TimeSeriesData {
 public:
  static constexpr std::uint32_t kSecond2Ns{1000000000};

  CommonMsgData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit CommonMsgData(std::int64_t timestamp) noexcept;
  ~CommonMsgData() noexcept = default;
  CommonMsgData(CommonMsgData const&) = default;
  CommonMsgData& operator=(CommonMsgData const&) = default;
  CommonMsgData(CommonMsgData&&) = default;
  CommonMsgData& operator=(CommonMsgData&&) = default;

  /// @brief Get frame id
  ///
  /// @return frame_id_
  std::int64_t GetFrameId() const noexcept;

  /// @brief Set frame id
  ///
  /// @param frame_id  frame id
  void SetFrameId(std::int64_t frame_id) noexcept;

 private:
  /// @brief frame id
  std::int64_t frame_id_;
};

}

#endif // AUTODRIVE_TYPE_COMMON_HEADER_DATA_H_

/* vim: set ts=2 sw=2 sts=2 tw=100 et: */