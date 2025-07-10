//

#include "interface/serializer/transaction_serializer.h"
#include "serializer/common_serializer.h"
#include "interface/transaction_data.h"
#include "protocol/transaction.pb.h"

namespace autodrive {

int TransactionSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  proto_msg->set_topic(data_msg->GetTopic());
  proto_msg->set_data_frame_id(data_msg->GetDataFrameId());
  proto_msg->set_dag_id(data_msg->GetDagId());
  proto_msg->set_enter_time(data_msg->GetEnterTime());
  return 0;
}

int TransactionSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  data_msg->SetTopic(proto_msg->topic());
  data_msg->SetDataFrameId(proto_msg->data_frame_id());
  data_msg->SetDagId(proto_msg->dag_id());
  data_msg->SetEnterTime(proto_msg->enter_time());
  return 0;
}

}
