//
#ifndef AUTODRIVE_TYPE_PARSING_BASE_H_
#define AUTODRIVE_TYPE_PARSING_BASE_H_

#include <array>
#include <cstdint>
#include <cstring>

#include "perception_base.h"

namespace autodrive {

static constexpr std::uint32_t kMaxIPMParsingImageDataSize{200704U};  // 448*448
static constexpr std::uint32_t kMaxCylParsingDataSize{101376U};       // 288*352
static constexpr std::uint32_t kMaxRleLineSize{448U};

struct alignas(8) IpmParsingImageData {
  std::uint8_t data[kMaxIPMParsingImageDataSize];
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t channel;
  std::uint8_t padding[4];
};

// declaration: segment map max shape should not be larger than 288*352
struct alignas(8) CylParsingImageData {
  std::uint8_t data[kMaxCylParsingDataSize];
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t channel;
  std::uint8_t padding[4];
};

struct ConnectRegion;

struct alignas(8) Rle {
  std::int32_t y;
  std::int32_t xstart;
  std::int32_t xend;
  std::int16_t length;
  std::int16_t value;
  ConnectRegion *region{nullptr};
};

struct alignas(8) RleList {
  Rle irle;
  RleList *next{nullptr};
};

struct alignas(8) RleLine {
  RleList *head{nullptr};
  RleList *tail{nullptr};
};

struct alignas(8) ConnectRegion {
  BbRect boundbox;
  std::int32_t area;
  std::int32_t pool_idx;
  RleList *rle_head;
  RleList *rle_tail;

  std::int16_t num_line;
  std::array<RleLine, kMaxRleLineSize> line_ptr;

  ConnectRegion *pre;
  ConnectRegion *next;

  ConnectRegion()
      : boundbox{0, 0, 0, 0},
        area(0),
        pool_idx(0),
        rle_head(nullptr),
        rle_tail(nullptr),
        num_line(0),
        pre(nullptr),
        next(nullptr) {}

  ConnectRegion& operator=(const ConnectRegion& c) {
    if(this != &c) {
      int tmp = this->pool_idx;
      memcpy((void*)this, (void*)&c, sizeof(c));
      this->pool_idx = tmp;
    }
    return *this;
  }
  ConnectRegion(const ConnectRegion& c) = delete;
};

struct alignas(8) RleCompressedData {
  /// @brief size of parsing img after compress
  std::uint32_t num_data{0};

  /// @brief pointer to the start of rle list
  RleList* rle_list_buf{nullptr};

  /// @brief length of rle list
  std::uint32_t num_rle_list_buf{0};
};

}  // namespace autodrive

#endif
