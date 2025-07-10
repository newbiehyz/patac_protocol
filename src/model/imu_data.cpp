//
#include "model/imu_data.h"

#include <cstring>
// #include <utils/timer.h>
// #include <utils/exception.h>

namespace autodrive {

ImuData::ImuData() noexcept : CommonMsgData{}, imu_{} {
}

ImuData::ImuData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp}, imu_{} {
}

ImuData::Imu const& ImuData::GetImu() const noexcept {
  return imu_;
}

ImuData::Imu& ImuData::MutableImu() noexcept {
  return imu_;
}

void ImuData::SetImu(Imu const& imu) noexcept {
  imu_ = imu;
}

void ImuData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&imu_, 0, sizeof(imu_));
}

// void ImuData::FillData(imu_comm::ImuComm* comm_ptr) noexcept(false) {
//   if (comm_ptr == nullptr) {
//     throw InvalidArgumentException("Imu comm is null");
//   }
//   Clear();
//   SetRawTimestamp(static_cast<std::int64_t>(comm_ptr->high_timestamp) * kSecond2Ns +
//       comm_ptr->low_timestamp);
//   SetTimestamp(Timer::Now());
//   memcpy(&imu_, &(comm_ptr->imu), sizeof(comm_ptr->imu));
// }

// std::uint32_t ImuData::FillCommData(void* data_buf, std::uint32_t nbytes)
//     noexcept(false) {
//   if (nbytes < sizeof(imu_comm::ImuComm)) {
//     throw OutOfResourceException("Imu data too large");
//   }
//   auto comm_ptr{static_cast<imu_comm::ImuComm*>(data_buf)};
//   comm_ptr->high_timestamp = static_cast<std::uint32_t>(
//       GetTimestamp() / kSecond2Ns);
//   comm_ptr->low_timestamp = static_cast<std::uint32_t>(
//       GetTimestamp() % kSecond2Ns);
//   memcpy(&(comm_ptr->imu), &imu_, sizeof(imu_));
//   return sizeof(imu_comm::ImuComm);
// }

}