//
#include "interface/parking/serializer/parking_slots_serializer.h"

#include <utils/exception.h>

#include "interface/parking/parking_slots_data.h"
#include "protocol/parking_slots.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/parking_slot_serializer.h"

namespace autodrive {

int ParkingSlotsSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto park_slots_data{data_msg->GetParkingSlots()};

  for (std::uint32_t psi_idx = 0; psi_idx < park_slots_data.num_parking_slots;
       psi_idx++) {
    const auto& slot_info_data{park_slots_data.parking_slots.at(psi_idx)};
    workflow::proto::parking_slot::ParkingSlotInfo* slot_info_proto =
        proto_msg->add_parking_slots();

    ParkingSlotSerialize(slot_info_data, slot_info_proto);
  }
  return 0;
}

int ParkingSlotsSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& slot_Info_data{data_msg->MutableParkingSlots()};
  slot_Info_data.num_parking_slots = proto_msg->parking_slots_size();

  for (std::uint32_t psl_idx = 0; psl_idx < slot_Info_data.num_parking_slots;
       psl_idx++) {
    auto& slot_data{slot_Info_data.parking_slots.at(psl_idx)};
    const auto& slot_proto{proto_msg->parking_slots(psl_idx)};

    ParkingSlotDeserialize(slot_proto, slot_data);
  }
  return 0;
}

}  // namespace autodrive