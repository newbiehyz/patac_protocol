//

#ifndef AUTODRIVE_TYPE_SERIALIZER_PARKING_SLOT_SERIALIZER_H_
#define AUTODRIVE_TYPE_SERIALIZER_PARKING_SLOT_SERIALIZER_H_

#include "protocol/parking_slot.pb.h"
#include "type/parking_slot.h"

namespace autodrive {

int ParkingSlotSerialize(
    const ParkingSlotInfo& slot_data,
    workflow::proto::parking_slot::ParkingSlotInfo* slot_proto);

int ParkingSlotDeserialize(
    const workflow::proto::parking_slot::ParkingSlotInfo& slot_proto,
    ParkingSlotInfo& slot_data);

}  // namespace autodrive

#endif  // AUTODRIVE_TYPE_SERIALIZER_PARKING_SLOT_SERIALIZER_H_