//
#ifndef AUTODRIVE_INTERFACE_IMAGE_DATA_H_
#define AUTODRIVE_INTERFACE_IMAGE_DATA_H_

#include <array>

#include <foundation/type/common_msg_data.h>

namespace autodrive {

class ImageData final : public CommonMsgData {
 public:
  /// @brief Image info
  struct alignas(8) ImageInfo {
    /// @brief Image width
    uint32_t width;
    /// @brief Image height
    uint32_t height;
  };

  /// @brief Info of image data in memio
  struct alignas(8) MemInfo {
    /// @brief Image data start addr
    std::uint64_t addr;
    /// @brief Image data size
    std::uint32_t size;
    /// @brief Image data pitch
    std::uint32_t pitch;
    std::uint8_t padding[4];
  };

  /// @brief Image data
  struct alignas(8) Image {
    /// @brief Image info
    ImageInfo image_info;
    /// @brief Info of image y data in memio
    MemInfo mem_info_y;
    /// @brief Info of image uv data in memio
    MemInfo mem_info_uv;
  };

  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ImageData(std::int64_t timestamp) noexcept;

  ImageData() noexcept;
  ~ImageData() noexcept final = default;
  ImageData(ImageData const&) = default;
  ImageData& operator=(ImageData const&) = default;
  ImageData(ImageData&&) = default;
  ImageData& operator=(ImageData&&) = default;

  /// @brief Get image_data_
  ///
  /// @return image_data_
  Image const& GetImageData() const noexcept;

  /// @brief Mutable image_data_
  ///
  /// @return image_data_
  Image& MutableImageData() noexcept;

  /// @brief Set image_data_
  ///
  /// @return void
  void SetImageData(Image const& image) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  Image image_data_;
};

}  // namespace autodrive

#endif
