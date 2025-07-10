//

#include "interface/parking/serializer/hmi_hpa_render_all_map_serializer.h"

#include "interface/parking/hmi_hpa_render_all_map_data.h"
#include "protocol/hmi_hpa_render_all_map.pb.h"
#include "protocol/map_display.pb.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>

namespace autodrive {

int HmdMapCommonHeaderDeserialize(
  const workflow::proto::HmdMapCommonHeader& map_header_proto,
  HmiMapDisplayData::MapCommonHeader& map_header_data);

int HmdMapCommonHeaderSerialize(
  const HmiMapDisplayData::MapCommonHeader& map_header_data, 
  workflow::proto::HmdMapCommonHeader* map_header_proto);    

int HmiHpaRenderAllMapSerializer::Serialize(const message::SerializeMsg *data_ptr,
                                      message::DeserializeMsg *msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto &hmi_hpa_render{data_msg->GetHmiHpaAllMapRender()};

  auto *hmi_hpa_proto =
      proto_msg->mutable_hmi_hpa_render();


  {
    auto map_display_proto = hmi_hpa_proto->mutable_hmi_map_display();
    auto &map_display_data{hmi_hpa_render.hmi_map_display};
    {
      auto *map_info_proto =
          map_display_proto->mutable_map_info();
      auto &map_info_data{map_display_data.map_info};
      {
        auto &header_data{map_info_data.header};
        auto *header_proto =
            map_info_proto->mutable_header();
        HmdMapCommonHeaderSerialize(header_data, header_proto);
      }  // end header

      {
        map_info_proto->set_map_distance(map_info_data.map_distance);
      }  // end map_distance

      {
        map_info_proto->set_rest_map_distance(map_info_data.rest_map_distance);
      }  // end rest_map_distance

      {
        map_info_proto->set_num_speed_bumps(map_info_data.num_speed_bumps);
      }  // end num_speed_bumps
    }  // end map_info

    map_display_proto->set_update_map_info(map_display_data.update_map_info);
    map_display_proto->set_is_match(map_display_data.is_match);
    map_display_proto->set_num_maps(static_cast<int32_t>(map_display_data.num_maps));
    map_display_proto->set_map_id(static_cast<int32_t>(map_display_data.map_id));


    for (std::uint8_t map_idx = 0; map_idx < map_display_data.num_maps;
         map_idx++) {
      auto &map_header_data{map_display_data.all_map_headers.at(map_idx)};
      auto *map_header_proto =
          map_display_proto->add_all_map_headers();
      HmdMapCommonHeaderSerialize(map_header_data, map_header_proto);
    } // end all_map_headers - for

  }
  return 0;
}

int HmiHpaRenderAllMapSerializer::Deserialize(const message::DeserializeMsg *msg_ptr,
                                        message::SerializeMsg *data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  auto &hmi_hpa_render{data_msg->MutableHmiHpaAllMapRender()};
  auto &hmi_hpa_proto = proto_msg->hmi_hpa_render();


  {
    auto map_display_proto = &(hmi_hpa_proto.hmi_map_display());
    auto &map_display_data = hmi_hpa_render.hmi_map_display;
    {
      if (map_display_proto->has_map_info()) {
        auto &map_info_data{map_display_data.map_info};
        auto &map_info_proto{map_display_proto->map_info()};
        if (map_info_proto.has_header()) {
          auto &header_data{map_info_data.header};
          auto &header_proto{map_info_proto.header()};
          HmdMapCommonHeaderDeserialize(header_proto, header_data);
        }  // end header

        {
          map_info_data.map_distance = map_info_proto.map_distance();
        }  // end map_distance

        {
          map_info_data.rest_map_distance = map_info_proto.rest_map_distance();
        }  // end rest_map_distance

        {
          map_info_data.num_speed_bumps = map_info_proto.num_speed_bumps();
        }  // end num_
      }  // end map_info

      map_display_data.update_map_info = map_display_proto->update_map_info();
      map_display_data.is_match = map_display_proto->is_match();
      map_display_data.map_id = static_cast<std::int32_t>(map_display_proto->map_id());

      map_display_data.num_maps =
          static_cast<std::int8_t>(map_display_proto->all_map_headers_size());
      for (std::int8_t idx = 0; idx < map_display_data.num_maps; idx++) {
        auto &map_header_data{map_display_data.all_map_headers.at(idx)};
        auto &map_header_proto{map_display_proto->all_map_headers(idx)};
        HmdMapCommonHeaderDeserialize(map_header_proto, map_header_data);
      }                                  
    }
  }
  return 0;
}
} // namespace autodrive
