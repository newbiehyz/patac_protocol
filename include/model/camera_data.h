//
#ifndef AUTODRIVE_MODEL_CAMERA_DATA_H_
#define AUTODRIVE_MODEL_CAMERA_DATA_H_

#include <cstdint>
#include <memory>
#include <array>

#include <foundation/type/common_msg_data.h>

namespace autodrive {

// Camera data
class CameraData final: public CommonMsgData {
 public:
  /// @brief Max image size
  static constexpr std::uint32_t kMaxImageSize{1500000U};

  // Position
  enum class Position : std::int8_t {
    UPLEFT = 0,
    UPRIGHT = 1,
    LOWERLEFT = 2,
    LOWERRIGHT = 3,
    AROUND_LEFT = 4,
    AROUND_RIGHT = 5,
    AROUND_FRONT = 6,
    AROUND_REAR = 7,
    FRONT = 8,
    REAR = 9
  };

  // Mode
  enum class Mode : std::int8_t {
    ORIGINAL = 0,
    STITCH = 1,
    CYL = 2
  };

  // type
  enum class Type : std::int8_t {
    H265 = 0,
    YUV420P = 1,
    BGR = 2,
    RGB = 3,
    JPEG = 4,
    H264 = 5
  };

  // Image
  struct Image {
    /// @brief Raw image data
    std::array<std::uint8_t, kMaxImageSize> data;
    /// @brief Width
    std::uint16_t width;
    /// @brief Height
    std::uint16_t height;
    /// @brief Element size
    std::uint8_t element_size;
    /// @brief Image pointer (for direct lvds access)
    void* data_ptr;
    /// @brief Data size (for compressed image)
    std::uint32_t data_size;
    /// @brief Position
    Position pos;
    /// @brief Mode
    Mode mode;
    /// @brief Type
    Type type;
  };

  CameraData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit CameraData(std::int64_t timestamp) noexcept;
  ~CameraData() noexcept = default;
  CameraData(CameraData const&) = default;
  CameraData& operator=(CameraData const&) = default;
  CameraData(CameraData&&) = default;
  CameraData& operator=(CameraData&&) = default;

  /// @brief Get image
  ///
  /// @return image_
  Image const& GetImage() const noexcept;

  /// @brief Mutable image
  ///
  /// @return image_
  Image& MutableImage() noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  /// @brief Image
  Image image_;
};

}

#endif
