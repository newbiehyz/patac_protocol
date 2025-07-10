//
#ifndef AUTODRIVE_TYPE_USS_BASE_H_
#define AUTODRIVE_TYPE_USS_BASE_H_

#include <cstdint>

namespace autodrive {

enum class UssHeaderGroupIdEn : std::uint8_t {
  kUssHeaderGroupIdEnNone = 0,
  kUssHeaderGroupIdEn1 = 1,
  kUssHeaderGroupIdEn2 = 2,
  kUssHeaderGroupIdEn3 = 3,
  kUssHeaderGroupIdEn4 = 4,
  kUssHeaderGroupIdEnMax = 255
};

enum class FreqModeEn : std::uint8_t {
  kFreqModeEnNone = 0,
  kFreqModeEnFixed = 1,
  kFreqModeEnChirp = 2,
  kFreqModeEnMax = 255
};

struct alignas(8) PerFusUssHeader {
  std::int64_t mcu_send_stamp_si64;
  std::int64_t mcu_send_wave_stamp_si64;

  std::uint16_t mesa_group_tx_mask_ui16;
  std::uint16_t mesa_group_rx_mask_ui16;

  UssHeaderGroupIdEn uss_header_group_id;
  FreqModeEn uss_header_freq_mode;
  std::uint8_t padding[2];
};

struct alignas(8) UssPointSi32 {
  std::int32_t x_si32;
  std::int32_t y_si32;
};

struct alignas(8) UssPointSi16 {
  std::int16_t X_si16;
  std::int16_t Y_si16;
  std::uint8_t padding[4];
};

enum class PlvState : std::uint8_t {
  kPlvStateNone = 0,
  kPlvStateOff = 1,
  kPlvStateActive = 2,
  kPlvStateMax = 255
};

enum class PsuState : std::uint8_t {
  kPsuStateNone = 0,
  kPsuStateOff = 1,
  kPsuStatePiActive = 2,
  kPsuStatePoActive = 3,
  kPsuStateMax = 255
};

enum class OdoState : std::uint8_t {
  kOdoStateNone = 0,
  kOdoStateReset = 1,
  kOdoStateActive = 2,
  kOdoStateResetSuccess = 3,
  kOdoStateMax = 255
};

struct alignas(8) StateMachine {
  PlvState plv_state;
  PsuState psu_state;
  OdoState odo_state;
  std::uint8_t padding[5];
};

}  // namespace autodrive

#endif
