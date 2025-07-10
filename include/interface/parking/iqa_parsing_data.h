//
#ifndef AUTODRIVE_INTERFACE_IQA_PARSING_DATA_H_
#define AUTODRIVE_INTERFACE_IQA_PARSING_DATA_H_

#include <foundation/type/common_msg_data.h>

#include <array>

#include "type/parsing_base.h"

#include "type/perception_base.h"

namespace autodrive {

class IqaParsingData final : public CommonMsgData {
 public:
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit IqaParsingData(std::int64_t timestamp) noexcept;

  /// @brief Max camera size
  static constexpr std::uint8_t kMaxCameraSize{5U};

  struct alignas(8) IqaParsing{
    ImageId image_id;
    std::uint8_t padding[7];
    CylParsingImageData parsing_data;
  };

  struct alignas(8) IqaParsingResults {
    std::uint8_t num_iqa_parsing_result;
    std::uint8_t padding[7];
    std::array<IqaParsing, kMaxCameraSize> iqa_parsing_result_data;
  };

  struct alignas(8) SceneParsingResults {
    std::uint8_t num_scene_parsing_result;
    std::uint8_t padding[7];
    std::array<autodrive::SceneParsing, kMaxCameraSize> scene_parsing_result_data;
  };

  IqaParsingData() noexcept;
  ~IqaParsingData() noexcept final = default;
  IqaParsingData(IqaParsingData const&) = default;
  IqaParsingData& operator=(IqaParsingData const&) = default;
  IqaParsingData(IqaParsingData&&) = default;
  IqaParsingData& operator=(IqaParsingData&&) = default;
  /// @brief Get Iqa Parsing data
  ///
  /// @return iqa_parsing_data_
  IqaParsingResults const& GetIqaParsingResults() const noexcept;

  /// @brief Get Scene Parsing data
  ///
  /// @return scene_parsing_data_
  SceneParsingResults const& GetSceneParsingResults() const noexcept;

  /// @brief Mutable Iqa Parsing data
  ///
  /// @return iqa_parsing_data_
  IqaParsingResults& MutableIqaParsingResults() noexcept;

  /// @brief Mutable Scene Parsing data
  ///
  /// @return scene_parsing_data_
  SceneParsingResults& MutableSceneParsingResults() noexcept;

  /// @brief Set Iqa Parsing data
  ///
  /// @param iqa_parsing_data Iqa Parsing data
  void SetIqaParsingResults(IqaParsingResults const& iqa_parsing_data) noexcept;

  /// @brief Set Scene Parsing data
  ///
  /// @param scene_parsing_data Scene Parsing data
  void SetSceneParsingResults(SceneParsingResults const& scene_parsing_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  IqaParsingResults iqa_parsing_data_;
  SceneParsingResults scene_parsing_data_;
};

}  // namespace autodrive

#endif
