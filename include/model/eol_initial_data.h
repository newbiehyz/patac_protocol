//
#ifndef AUTODRIVE_INTERFACE_EOL_INITIAL_DATA_H_
#define AUTODRIVE_INTERFACE_EOL_INITIAL_DATA_H_

#include <foundation/type/common_msg_data.h>

#include "model/camera_extrinsics_data.h"

namespace autodrive {

class EolInitialData final : public CommonMsgData {
 public:
  static constexpr std::uint8_t kMaxCameraSize{5U};
  static constexpr std::uint8_t kMaxSVCSize{4U};
  static constexpr std::uint8_t kMaxRLCSize{1U};

  struct alignas(8) ChessboardData {
    CameraID camera_id;
    std::uint8_t padding[7];
    float top_left_x;
    float top_left_y;
    float height_interval;
    float width_interval;
  };

  struct alignas(8) CircleGridData {
    CameraID camera_id;
    std::uint8_t padding[3];
    float basic_pt_x;
    float basic_pt_y;
    float basic_pt_z;
    float height_interval;
    float width_interval;
  };

  struct alignas(8) VehicleExtrinsicData {
    ///@brief the distance between front axle and rear axle
    float wheelbase;
    std::uint32_t num_ori_extrinsic;
    ///@brief include all camera's initial extrinsic params
    std::array<CameraExtrinsicsData::Extrinsic, kMaxCameraSize> ori_extrinsic;
  };

  struct alignas(8) EolInitial {
    VehicleExtrinsicData vehicle_data;
    std::uint32_t num_cb_data;
    std::uint32_t num_cg_data;
    std::array<ChessboardData, kMaxSVCSize> cb_data;
    std::array<CircleGridData, kMaxRLCSize> cg_data;
  };

  EolInitialData() noexcept;
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit EolInitialData(std::int64_t timestamp) noexcept;
  ~EolInitialData() noexcept final = default;
  EolInitialData(EolInitialData const&) = default;
  EolInitialData& operator=(EolInitialData const&) = default;
  EolInitialData(EolInitialData&&) = default;
  EolInitialData& operator=(EolInitialData&&) = default;

  EolInitial const& GetInitialData() const noexcept;
  EolInitial& MutableInitialData() noexcept;
  void SetInitialData(EolInitial const& initial_data) noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  EolInitial initial_data_;
};
}  // namespace autodrive

#endif