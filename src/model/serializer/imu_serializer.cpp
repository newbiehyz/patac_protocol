//
#include "model/serializer/imu_serializer.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "model/imu_data.h"
#include "protocol/imu.pb.h"

namespace autodrive {

int ImuSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto &proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto &data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  workflow::proto::Imu* imu{proto_msg->mutable_imu()};
  imu->set_utc_time(data_msg->GetImu().utc_time);
  imu->set_heading(data_msg->GetImu().heading);
  imu->set_pitch(data_msg->GetImu().pitch);
  imu->set_roll(data_msg->GetImu().roll);
  imu->set_raw_acceleration_x(data_msg->GetImu().raw_acceleration_x);
  imu->set_raw_acceleration_y(data_msg->GetImu().raw_acceleration_y);
  imu->set_raw_acceleration_z(data_msg->GetImu().raw_acceleration_z);
  imu->set_raw_angular_rate_x(data_msg->GetImu().raw_angular_rate_x);
  imu->set_raw_angular_rate_y(data_msg->GetImu().raw_angular_rate_y);
  imu->set_raw_angular_rate_z(data_msg->GetImu().raw_angular_rate_z);
  return 0;
}

int ImuSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  ImuData::Imu& imu{data_msg->MutableImu()};
  imu.utc_time = proto_msg->imu().utc_time();
  imu.heading = proto_msg->imu().heading();
  imu.pitch = proto_msg->imu().pitch();
  imu.roll = proto_msg->imu().roll();
  imu.raw_acceleration_x = proto_msg->imu().raw_acceleration_x();
  imu.raw_acceleration_y = proto_msg->imu().raw_acceleration_y();
  imu.raw_acceleration_z = proto_msg->imu().raw_acceleration_z();
  imu.raw_angular_rate_x = proto_msg->imu().raw_angular_rate_x();
  imu.raw_angular_rate_y = proto_msg->imu().raw_angular_rate_y();
  imu.raw_angular_rate_z = proto_msg->imu().raw_angular_rate_z();
  return 0;
}

}