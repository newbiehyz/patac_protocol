#ifndef PROTOCOL_UTILS_RLE_UTILS_
#define PROTOCOL_UTILS_RLE_UTILS_

#include "type/parsing_base.h"

#include <vector>

namespace autodrive {

class RleUtils {
public:
  /// @brief Writr RleList data to continuous buffer
  ///
  /// @param rle_list_ptr ptr to the start of rle list
  /// @param rle_list_num size of rle list
  /// @param width width of parsing image
  /// @param height height of parsing image
  /// @param buf_ptr ptr to the output buffer
  ///
  /// @throw
  static void WriteToBuffer(RleList const* rle_list_ptr, uint32_t rle_list_num,
                           uint32_t width, uint32_t height, uint8_t* buf_ptr);

  enum {
    Decompress_OK = 0,
    Decompress_Failed = -1,
    Decompress_BuffTooSmall = -2
  };

  /// @brief Decompress rle compressed data to original parsing image
  ///
  /// @param buffer_ptr ptr to rle buffer
  /// @param buffer_num size of buffer
  /// @param parsing_ptr ptr to the parsing image
  /// @param parsing_img_sz parsing image width * height
  /// @param align default is zero, used to add padding to parsing image
  ///
  /// @throw
  static int Decompress(uint8_t const* buffer_ptr, uint32_t buffer_num,
                        uint8_t *parsing_ptr, int parsing_img_sz,
                        uint16_t align = 0);

  /// @brief Compress parsing image to vector of RleList
  ///
  /// @param img ptr to parsing image
  /// @param width width of parsing image
  /// @param height height of parsing image
  /// @param rle_list_buf vector to store rle result
  /// @param rle_list_num ptr to size of rle list
  /// @param buf_data_num ptr to size of rle buffer
  ///
  /// @throw
  static void Compress(uint8_t const* img, uint32_t width, uint32_t height,
                       std::vector<RleList> & rle_list_buf,
                       uint32_t* rle_list_num = nullptr,
                       uint32_t* buf_data_num = nullptr);
            
};

}
#endif