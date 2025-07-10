
//
#ifndef AUTODRIVE_MESSAGE_DATA_REF_H
#define AUTODRIVE_MESSAGE_DATA_REF_H

#include <cstdint>

namespace autodrive {
namespace message {

class DataRef {
 public:
  struct RefData {
    std::uint8_t *data;
    std::uint32_t size;
  };

  DataRef(RefData const& ref_data) noexcept;
  DataRef() noexcept;
  virtual ~DataRef() = default;
  DataRef(const DataRef &) = default;
  DataRef &operator=(const DataRef &) = default;
  DataRef(DataRef &&) = default;
  DataRef &operator=(DataRef &&) = default;

  /// @brief Set data
  ///
  /// @return void
  void SetData(RefData const& ref_data) noexcept(false);

  /// @brief Get data
  ///
  /// @return data_
  RefData GetData() const noexcept(false);

 private:
  RefData ref_data_;
};

}
}

#endif