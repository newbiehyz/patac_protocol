//
#include "interface/parking/serializer/processor_info_serializer.h"

#include <utils/exception.h>

#include "interface/parking/processor_info_data.h"
#include "protocol/processor_info.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int ProcessorInfoSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                       message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  auto& process_info_data{data_msg->GetProcessorInfo()};
  proto_msg->set_status(
      static_cast<workflow::proto::ProcessorStatus>(process_info_data.status));
  proto_msg->set_error_code(
      static_cast<workflow::proto::ProcessorErrorCode>(process_info_data.error_code));

  return 0;
}

int ProcessorInfoSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto& process_info_data{data_msg->MutableProcessorInfo()};
  process_info_data.status =
      static_cast<ProcessorInfoData::ProcessorStatus>(proto_msg->status());
  process_info_data.error_code =
      static_cast<ProcessorInfoData::ErrorCode>(proto_msg->error_code());
  return 0;
}

}  // namespace autodrive