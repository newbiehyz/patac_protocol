#include "utils/rle_utils.h"

namespace autodrive
{

  void RleUtils::WriteToBuffer(RleList const *rle_list_ptr, uint32_t rle_list_num,
                              uint32_t width, uint32_t height,
                              uint8_t *buf_ptr)
  {

    // Put magic number (for image type ident) at the head
    *buf_ptr++ = 0xFF;
    *buf_ptr++ = static_cast<uint8_t>('R'); // R
    *buf_ptr++ = static_cast<uint8_t>('L'); // L
    *buf_ptr++ = static_cast<uint8_t>('E'); // E
    // Then the height and width
    uint32_t uint_height = static_cast<uint32_t>(height);
    uint32_t uint_width = static_cast<uint32_t>(width);
    *buf_ptr++ = static_cast<uint8_t>((uint_height >> 8) & 0xFF);
    *buf_ptr++ = static_cast<uint8_t>(uint_height & 0xFF);
    *buf_ptr++ = static_cast<uint8_t>((uint_width >> 8) & 0xFF);
    *buf_ptr++ = static_cast<uint8_t>(uint_width & 0xFF);

    // Then the data
    for (size_t i = 0; i < rle_list_num; ++i)
    {
      uint8_t value = static_cast<uint8_t>(rle_list_ptr[i].irle.value);
      uint16_t len = rle_list_ptr[i].irle.length;
      *buf_ptr++ = value;
      *reinterpret_cast<uint16_t *>(buf_ptr) = len;
      buf_ptr += 2;
    }
  }

  int RleUtils::Decompress(uint8_t const *buffer_ptr, uint32_t buffer_num,
                           uint8_t *parsing_ptr, int parsing_img_sz,
                           uint16_t align)
  {
    if (!buffer_ptr || buffer_num == 0)
    {
      return Decompress_Failed;
    }

    // Skip the magic number first
    buffer_ptr += 4;
    // Then the height and width
    uint16_t height =
        static_cast<uint16_t>(static_cast<uint16_t>(*buffer_ptr++ << 8) & 0xFF00);
    height |= static_cast<uint16_t>(*buffer_ptr++);
    uint16_t width =
        static_cast<uint16_t>(static_cast<uint16_t>(*buffer_ptr++ << 8) & 0xFF00);
    width |= static_cast<uint16_t>(*buffer_ptr++);

    if (height <= 0 || width <= 0)
    {
      return Decompress_Failed;
    }

    uint16_t step = width;
    uint16_t padding_value = 0;
    if (align > 0)
    {
      step = (width + align - 1) / align * align;
      padding_value = step - width;
    }

    if (parsing_img_sz < height * step)
    {
      return Decompress_BuffTooSmall;
    }

    // buf_len = height * step;

    // Then the data
    int x = 0;
    int y = 0;
    int rle_cnt = (buffer_num - 8) / 3;
    int offset = 0;
    for (int i = 0; i < rle_cnt; ++i)
    {
      uint8_t label = *buffer_ptr++;
      uint16_t cnt = *reinterpret_cast<const uint16_t *>(buffer_ptr);
      buffer_ptr += 2;

      x += cnt;
      if (x > width)
      {
        return Decompress_Failed;
      }
      if (y > height)
      {
        return Decompress_Failed;
      }
      offset += static_cast<int>(cnt);
      if (offset > parsing_img_sz)
      {
        return Decompress_Failed;
      }

      for (int j = 0; j < cnt; ++j)
      {
        *parsing_ptr++ = label;
      }

      if (x == width)
      {
        y++;
        x = 0;
        parsing_ptr += padding_value;
        offset += padding_value;
        if (offset > parsing_img_sz)
        {
          return Decompress_Failed;
        }
      }
    }

    return Decompress_OK;
  }

  void RleUtils::Compress(uint8_t const* img, uint32_t width, uint32_t height,
                       std::vector<RleList> & rle_list_buf,
                       uint32_t* rle_list_num,
                       uint32_t* buf_data_num) {
  rle_list_buf.resize(10000);
  rle_list_buf.clear();
  int step = width;
  int rle_list_cnt = 0;
  for (int16_t y = 0; y < height; y++) {
    uint8_t const* ptr = img + y * step;
    for (int16_t x = 0; x < width; x++) {
      if (rle_list_cnt >= rle_list_buf.size()) {
        rle_list_buf.resize(rle_list_buf.size() + 2000);
      }
      RleList &rlist = rle_list_buf[rle_list_cnt++];
      rlist.next = NULL;
      Rle *rle = &rlist.irle;
      rle->y = y;
      rle->xstart = x;
      // rle->xend = width - 1;
      // rle->length = 0;
      rle->value = ptr[x];
      rle->region = NULL;
      int16_t cx = x + 1;
      for (; cx < width; cx++) {
        if (*(ptr + cx) != rle->value) {
          break;
        }
      }
      rle->xend = cx - 1;
      rle->length = static_cast<int16_t>(rle->xend - rle->xstart + 1);
      x = rle->xend;
    }
  }
  if(rle_list_num) 
    *rle_list_num = rle_list_cnt;
  if(buf_data_num) 
    *buf_data_num = 3 * static_cast<int>(rle_list_cnt) + 8;
}
}
