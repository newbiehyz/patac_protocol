//
#include "serializer/common_serializer.h"

namespace autodrive {

int CommonSerializer::Serialize(const CommonMsgData* data_com, workflow::proto::MsgHeader* proto_hdr) {
  proto_hdr->set_frame_id(data_com->GetFrameId());
  proto_hdr->set_local_timestamp(data_com->GetTimestamp());
  proto_hdr->set_raw_timestamp(data_com->GetRawTimestamp());
  return 0;
}

int CommonSerializer::Deserialize(const workflow::proto::MsgHeader* proto_hdr, CommonMsgData* data_com) {
  data_com->SetFrameId(proto_hdr->frame_id());
  data_com->SetTimestamp(proto_hdr->local_timestamp());
  data_com->SetRawTimestamp(proto_hdr->raw_timestamp());
  return 0;
}

}