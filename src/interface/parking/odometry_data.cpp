//
#include "interface/parking/odometry_data.h"

#include <cstring>

namespace autodrive {

OdometryData::OdometryData() noexcept : CommonMsgData{}, odometry_data_{} {}

OdometryData::OdometryData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, odometry_data_{} {}

OdometryData::Odometry const& OdometryData::GetOdometry() const noexcept {
  return odometry_data_;
}

OdometryData::Odometry& OdometryData::MutableOdometry() noexcept {
  return odometry_data_;
}

void OdometryData::SetOdometry(Odometry const& odometry_data) noexcept {
  odometry_data_ = odometry_data;
}

void OdometryData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&odometry_data_, 0, sizeof(odometry_data_));
}

}  // namespace autodrive