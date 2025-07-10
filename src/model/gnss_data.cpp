//
#include "model/gnss_data.h"

#include <cstring>
// #include <utils/timer.h>
// #include <utils/exception.h>

namespace autodrive {

GnssData:: GnssData() noexcept : CommonMsgData{}, gnss_{} {
}

GnssData:: GnssData(std::int64_t timestamp) noexcept :
    CommonMsgData{timestamp}, gnss_{} {
}

GnssData::Gnss const& GnssData::GetGnss() const noexcept {
  return gnss_;
}

 GnssData::Gnss&  GnssData::MutableGnss() noexcept {
  return gnss_;
}

void GnssData::SetGnss(Gnss const& gnss) noexcept {
  gnss_ = gnss;
}

void  GnssData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&gnss_, 0, sizeof(gnss_));
}

// void GnssData::FillData(gnss_comm::GnssComm* comm_ptr) noexcept(false) {
//   if (comm_ptr == nullptr) {
//     throw InvalidArgumentException("Vehicle comm is null");
//   }
//   Clear();
//   SetRawTimestamp(static_cast<std::int64_t>(comm_ptr->high_timestamp) * kSecond2Ns +
//       comm_ptr->low_timestamp);
//   SetTimestamp(Timer::Now());
//   memcpy(&gnss_, &(comm_ptr->gnss), sizeof(comm_ptr->gnss));
// }

// std::uint32_t GnssData::FillCommData(void* data_buf, std::uint32_t nbytes)
//     noexcept(false) {
//   if (nbytes < sizeof(gnss_comm::GnssComm)) {
//     throw OutOfResourceException("Gnss data too large");
//   }
//   auto comm_ptr{static_cast<gnss_comm::GnssComm*>(data_buf)};
//   comm_ptr->high_timestamp = static_cast<std::uint32_t>(
//       GetTimestamp() / kSecond2Ns);
//   comm_ptr->low_timestamp = static_cast<std::uint32_t>(
//       GetTimestamp() % kSecond2Ns);
//   memcpy(&(comm_ptr->gnss), &gnss_, sizeof(gnss_));
//   return sizeof(gnss_comm::GnssComm);
// }

}