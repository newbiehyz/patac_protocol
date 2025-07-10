//
#include "interface/parking/serializer/ipm_parsing_serializer.h"

#include <utils/exception.h>
#include "utils/rle_utils.h"

#include "interface/parking/ipm_parsing_data.h"
#include "protocol/ipm_parsing.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int IpmParsingSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& imp_parse_data{data_msg->GetIpmParsing()};
  proto_msg->set_scale(imp_parse_data.scale);
  {
    auto& free_space_data{imp_parse_data.ground_pt};
    const auto& rle_ground_pt = imp_parse_data.rle_ground_pt;
    uint32_t num_rle_list_buf_g = rle_ground_pt.num_rle_list_buf;
    uint32_t num_buf_data_g = rle_ground_pt.num_data;
    RleList* rle_list_buf_g = rle_ground_pt.rle_list_buf;
    std::vector<RleList> rle_list_buf_tmp_g;
    if(!rle_list_buf_g) {
        RleUtils::Compress(static_cast<std::uint8_t const*>(free_space_data.data), free_space_data.width, free_space_data.height,
                            rle_list_buf_tmp_g, &num_rle_list_buf_g, &num_buf_data_g);
        rle_list_buf_g = rle_list_buf_tmp_g.data();
    }
    IpmParsingImageData tmp_compressed_ground_pt;
    RleUtils::WriteToBuffer(rle_list_buf_g, num_rle_list_buf_g,
        free_space_data.width, free_space_data.height, static_cast<std::uint8_t*>(tmp_compressed_ground_pt.data));
    workflow::proto::IpmParsingImageData* free_space_proto =
        proto_msg->mutable_free_space();
    free_space_proto->set_data((char*)tmp_compressed_ground_pt.data, num_buf_data_g);
    free_space_proto->set_width(free_space_data.width);
    free_space_proto->set_height(free_space_data.height);
    free_space_proto->set_channel(free_space_data.channel);
  }  // end free_space

  {
    auto& feature_map_data{imp_parse_data.feature_map};
    const auto& rle_feature_map = imp_parse_data.rle_feature_map;
    uint32_t num_rle_list_buf_f = rle_feature_map.num_rle_list_buf;
    uint32_t num_buf_data_f = rle_feature_map.num_data;
    RleList* rle_list_buf_f = rle_feature_map.rle_list_buf;
    std::vector<RleList> rle_list_buf_tmp_f;
    if(!rle_list_buf_f) {
        RleUtils::Compress(static_cast<std::uint8_t const*>(feature_map_data.data), feature_map_data.width, feature_map_data.height,
                            rle_list_buf_tmp_f, &num_rle_list_buf_f, &num_buf_data_f);
        rle_list_buf_f = rle_list_buf_tmp_f.data();
    }
    IpmParsingImageData tmp_compressed_feature_map;
    RleUtils::WriteToBuffer(rle_list_buf_f, num_rle_list_buf_f,
        feature_map_data.width, feature_map_data.height, static_cast<std::uint8_t*>(tmp_compressed_feature_map.data));
    workflow::proto::IpmParsingImageData* feature_map_proto =
        proto_msg->mutable_feature_map();
    feature_map_proto->set_data((char*)tmp_compressed_feature_map.data, num_buf_data_f);
    feature_map_proto->set_width(feature_map_data.width);
    feature_map_proto->set_height(feature_map_data.height);
    feature_map_proto->set_channel(feature_map_data.channel);
  }
  workflow::proto::perception_base::Rect* rect =
      proto_msg->mutable_roi_output();
  rect->set_left(imp_parse_data.roi_output.left);
  rect->set_top(imp_parse_data.roi_output.top);
  rect->set_right(imp_parse_data.roi_output.right);
  rect->set_bottom(imp_parse_data.roi_output.bottom);

  return 0;
}

int IpmParsingSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                      message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& ipm_parse_data{data_msg->MutableIpmParsing()};

  if (proto_msg->has_free_space()) {
    auto& free_space_proto{proto_msg->free_space()};
    auto& free_space_data{ipm_parse_data.ground_pt};
    auto& str_data{free_space_proto.data()};

    if (str_data.length() > kMaxIPMParsingImageDataSize) {
      throw OutOfResourceException("ipm_parsing free_space_data->data info size too long");
    }

    free_space_data.width =
        static_cast<std::uint32_t>(free_space_proto.width());
    free_space_data.height =
        static_cast<std::uint32_t>(free_space_proto.height());
    free_space_data.channel =
        static_cast<std::uint32_t>(free_space_proto.channel());

    RleUtils::Decompress(reinterpret_cast<std::uint8_t const*>(str_data.c_str()), str_data.length(),
        static_cast<std::uint8_t*>(free_space_data.data), free_space_data.width * free_space_data.height);
  }  // end free space

  if (proto_msg->has_feature_map()) {
    auto& feature_map_proto{proto_msg->feature_map()};
    auto& feature_map_data{ipm_parse_data.feature_map};
    auto& str_data{feature_map_proto.data()};

    if (str_data.length() > kMaxIPMParsingImageDataSize) {
      throw OutOfResourceException("ipm_parsing feature_map_data->data info size too long");
    }
    feature_map_data.width =
        static_cast<std::uint32_t>(feature_map_proto.width());
    feature_map_data.height =
        static_cast<std::uint32_t>(feature_map_proto.height());
    feature_map_data.channel =
        static_cast<std::uint32_t>(feature_map_proto.channel());

    RleUtils::Decompress(reinterpret_cast<std::uint8_t const*>(str_data.c_str()), str_data.length(),
        static_cast<std::uint8_t*>(feature_map_data.data), feature_map_data.width * feature_map_data.height);
  }  // end feature_map

  ipm_parse_data.scale = proto_msg->scale();

  ipm_parse_data.roi_output.bottom = proto_msg->roi_output().bottom();
  ipm_parse_data.roi_output.top = proto_msg->roi_output().top();
  ipm_parse_data.roi_output.left = proto_msg->roi_output().left();
  ipm_parse_data.roi_output.right = proto_msg->roi_output().right();

  return 0;
}

}  // namespace autodrive