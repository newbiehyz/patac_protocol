
//
#ifndef AUTODRIVE_FABRIC_BUS_SERIALIZER_H_
#define AUTODRIVE_FABRIC_BUS_SERIALIZER_H_

#include <foundation/type/common_msg_data.h>
#include "protocol/common.pb.h"

namespace autodrive {

class CommonSerializer {
public:
  CommonSerializer() noexcept = default;
  ~CommonSerializer() noexcept = default;
  CommonSerializer(CommonSerializer const&) = default;
  CommonSerializer& operator=(CommonSerializer const&) = default;
  CommonSerializer(CommonSerializer&&) = default;
  CommonSerializer& operator=(CommonSerializer&&) = default;

  static int Serialize(const CommonMsgData* data_com, workflow::proto::MsgHeader* proto_hdr);
  static int Deserialize(const workflow::proto::MsgHeader* proto_hdr, CommonMsgData* data_com);
};

}

#endif