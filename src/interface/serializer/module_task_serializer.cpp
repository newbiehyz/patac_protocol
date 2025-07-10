//

#include "interface/serializer/module_task_serializer.h"
#include "serializer/common_serializer.h"
#include "interface/module_task_data.h"
#include "protocol/module_task.pb.h"

namespace autodrive {

int ModuleTaskSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  proto_msg->set_dag_id(data_msg->GetDagId());
  proto_msg->set_module_id(data_msg->GetModuleId());
  proto_msg->set_start_time(data_msg->GetStartTime());
  proto_msg->set_end_time(data_msg->GetEndTime());
  return 0;
}

int ModuleTaskSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  data_msg->SetDagId(proto_msg->dag_id());
  data_msg->SetModuleId(proto_msg->module_id());
  data_msg->SetStartTime(proto_msg->start_time());
  data_msg->SetEndTime(proto_msg->end_time());
  return 0;
}

}