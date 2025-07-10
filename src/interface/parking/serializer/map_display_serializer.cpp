//
#include "interface/parking/serializer/map_display_serializer.h"

#include <utils/exception.h>

#include "interface/parking/map_display_data.h"
#include "protocol/map_display.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/mapinfo_header_serialize.h"

namespace autodrive {

int MapDisplaSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                   message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  auto& map_display_data{data_msg->GetMapDisplayData()};
  {
    workflow::proto::MapInfo* map_info_proto = proto_msg->mutable_map_info();
    auto& map_info_data{map_display_data.map_info};
    autodrive::MapInfoSerialize(map_info_data, map_info_proto);
  }
    proto_msg->set_update_map_info(map_display_data.update_map_info);
  for (std::uint8_t map_idx = 0; map_idx < map_display_data.num_maps;
       map_idx++) {
    auto& map_header_data{map_display_data.all_map_headers.at(map_idx)};
    workflow::proto::MapCommonHeader* map_header_proto =
        proto_msg->add_all_map_headers();
    autodrive::MapCommonHeaderSerialize(map_header_data, map_header_proto);
  }  // end all_map_headers - for
  return 0;
}

int MapDisplaSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                     message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& map_display_data{data_msg->MutableMapDisplayData()};

  if (proto_msg->has_map_info()) {
    auto& map_info_data{map_display_data.map_info};
    auto& map_info_proto{proto_msg->map_info()};
    autodrive::MapInfoDeserialize(map_info_proto, map_info_data);
  }      // end map_info

  map_display_data.update_map_info = proto_msg->update_map_info();
  map_display_data.num_maps =
      static_cast<std::int8_t>(proto_msg->all_map_headers_size());

  for (std::int8_t idx = 0; idx < map_display_data.num_maps; idx++) {
    auto& map_header_data{map_display_data.all_map_headers.at(idx)};
    auto& map_header_proto{proto_msg->all_map_headers(idx)};
    autodrive::MapCommonHeaderDeserialize(map_header_proto, map_header_data);
  }  // end for - all_map_headers

  return 0;
}

}  // namespace autodrive