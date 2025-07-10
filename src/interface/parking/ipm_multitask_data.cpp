//

#include "interface/parking/ipm_multitask_data.h"

#include <cstring>

namespace autodrive {

IpmMultitaskData::IpmMultitaskData() noexcept
    : CommonMsgData{}, num_slots_raw_{0}, slots_raw_{} {}

IpmMultitaskData::IpmMultitaskData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp},
      num_slots_raw_{0},
      slots_raw_{} {}

std::array<IpmMultitaskData::ParkingSlotRaw,
           IpmMultitaskData::kMaxParkingSlotRawSize> const&
IpmMultitaskData::GetSlotsRaw() const noexcept {
  return slots_raw_;
}

std::array<IpmMultitaskData::ParkingSlotRaw,
           IpmMultitaskData::kMaxParkingSlotRawSize>&
IpmMultitaskData::MutableSlotsRaw() noexcept {
  return slots_raw_;
}

void IpmMultitaskData::SetSlotsRaw(
    std::array<ParkingSlotRaw, kMaxParkingSlotRawSize> const&
        slots_raw) noexcept {
  slots_raw_ = slots_raw;
}

uint8_t const& IpmMultitaskData::GetSlotsNum() const noexcept {
  return num_slots_raw_;
}

uint8_t& IpmMultitaskData::MutableSlotsNum() noexcept { return num_slots_raw_; }

void IpmMultitaskData::SetSlotsNum(const std::uint8_t& slots_num) noexcept {
  num_slots_raw_ = slots_num;
}

std::array<IpmMultitaskData::IpmWheelGroundPoint,
          IpmMultitaskData::kMaxIpmWheelGroundPointSize> const &
          IpmMultitaskData::GetIpmWheelGroundPoints()
     const noexcept {
    return ipm_wheel_ground_points_;
}

void IpmMultitaskData::SetIpmWheelGroundPoints(std::array<IpmMultitaskData::IpmWheelGroundPoint,
                            IpmMultitaskData::kMaxIpmWheelGroundPointSize> const
                      &ipm_wheel_ground_points) noexcept {
    ipm_wheel_ground_points_ = ipm_wheel_ground_points;
}

std::array<IpmMultitaskData::IpmWheelGroundPoint,
           IpmMultitaskData::kMaxIpmWheelGroundPointSize> &
  IpmMultitaskData::MutableIpmWheelGroundPoints() noexcept {
    return ipm_wheel_ground_points_;
}

void IpmMultitaskData::SetWheelGDPNum(const std::uint8_t& wheel_gdp_num) noexcept {
  num_wheel_gdp_ = wheel_gdp_num;
}

uint8_t const &IpmMultitaskData::GetWheelGDPNum() const noexcept {
  return num_wheel_gdp_;
}

uint8_t &IpmMultitaskData::MutableWheelGDPNum() noexcept{
  return num_wheel_gdp_;
}



float const &IpmMultitaskData::GetTest() const noexcept {
  return test_;
}
void IpmMultitaskData::SetTest(const float& test) noexcept {
  test_ = test;
}
float &IpmMultitaskData::MutableTest() noexcept {
  return test_;
}



/// @brief Clear data
void IpmMultitaskData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&slots_raw_, 0, sizeof(slots_raw_));
}

}  // end namespace autodrive