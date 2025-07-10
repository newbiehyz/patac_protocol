//
#ifndef AUTODRIVE_TIME_SERIES_DATA_H_
#define AUTODRIVE_TIME_SERIES_DATA_H_

#include <cstdint>
#include <fstream>

namespace autodrive {

// Time-series data
class TimeSeriesData {
 public:
  TimeSeriesData() noexcept;
  explicit TimeSeriesData(std::int64_t timestamp) noexcept;
  virtual ~TimeSeriesData() noexcept = default;
  TimeSeriesData(TimeSeriesData const&) = default;
  TimeSeriesData& operator=(TimeSeriesData const&) = default;
  TimeSeriesData(TimeSeriesData&&) = default;
  TimeSeriesData& operator=(TimeSeriesData&&) = default;

  /// @brief Get time stamp
  ///
  /// @return timestamp_
  std::int64_t GetTimestamp() const noexcept;

  /// @brief Set time stamp
  ///
  /// @param timestamp  time stamp
  void SetTimestamp(std::int64_t timestamp) noexcept;

  /// @brief Get time stamp
  ///
  /// @return timestamp_
  std::int64_t GetRawTimestamp() const noexcept;

  /// @brief Set time stamp
  ///
  /// @param timestamp  time stamp
  void SetRawTimestamp(std::int64_t timestamp) noexcept;

 private:
  /// @brief Timestamp
  std::int64_t local_timestamp_;
  /// @brief Timestamp
  std::int64_t raw_timestamp_;
};

}

#endif // AUTODRIVE_SENSOR_MODEL_TIME_SERIES_DATA_H_

/* vim: set ts=2 sw=2 sts=2 tw=100 et: */
