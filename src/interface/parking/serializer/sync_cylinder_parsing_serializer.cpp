//
#include "interface/parking/serializer/sync_cylinder_parsing_serializer.h"

#include <utils/exception.h>
#include "utils/rle_utils.h"

#include "interface/parking/sync_cylinder_parsing_data.h"
#include "protocol/sync_cylinder_parsing.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int SyncCylinderParsingSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& sync_cylinder_data{data_msg->GetSyncCylinderParsing()};

  for (std::uint32_t scp_idx = 0;
       scp_idx < sync_cylinder_data.num_cylinder_parsing_data; scp_idx++) {
    auto& scp_data{sync_cylinder_data.cylinder_parsing_data.at(scp_idx)};
    workflow::proto::CylinderParsingData* scp_proto =
        proto_msg->add_cylinder_parsing_data();

    scp_proto->set_camera_idx(scp_data.camera_idx);
    scp_proto->set_scale(scp_data.scale);

    {
      auto& feature_map_data{scp_data.feature_map};
      const auto& rle_feature_map = scp_data.rle_feature_map;
      uint32_t num_rle_list_buf_f = rle_feature_map.num_rle_list_buf;
      uint32_t num_buf_data_f = rle_feature_map.num_data;
      auto rle_list_buf_f = rle_feature_map.rle_list_buf;
      std::vector<RleList> rle_list_buf_tmp;
      if(!rle_list_buf_f) {
        RleUtils::Compress(static_cast<std::uint8_t const*>(feature_map_data.data), feature_map_data.width, feature_map_data.height,
                           rle_list_buf_tmp, &num_rle_list_buf_f, &num_buf_data_f);
        rle_list_buf_f = rle_list_buf_tmp.data();
      }
      CylParsingImageData tmp_compressed_feature_map;
      RleUtils::WriteToBuffer(rle_list_buf_f, num_rle_list_buf_f,
          feature_map_data.width, feature_map_data.height, static_cast<std::uint8_t*>(tmp_compressed_feature_map.data));
      workflow::proto::CylinderParsingImageData* feature_map_proto =
          scp_proto->mutable_feature_map();
      feature_map_proto->set_data((char*)tmp_compressed_feature_map.data, num_buf_data_f);
      feature_map_proto->set_width(feature_map_data.width);
      feature_map_proto->set_height(feature_map_data.height);
      feature_map_proto->set_channel(feature_map_data.channel);
    }  // end feature_map

      SyncCylinderParsingData cyl_parsing_data2;
      message::SerializeDataMsg<SyncCylinderParsingData> data_msg2(&cyl_parsing_data2);
      SyncCylinderParsingSerializer serializer2;
      serializer2.Deserialize(msg_ptr, &data_msg2);
    {
      auto& roi_data{scp_data.roi_output};
      workflow::proto::perception_base::Rect* roi_proto =
          scp_proto->mutable_roi_output();
      roi_proto->set_bottom(roi_data.bottom);
      roi_proto->set_right(roi_data.right);
      roi_proto->set_left(roi_data.left);
      roi_proto->set_top(roi_data.top);
    }  // end roi_output
  }    // end cylinder_parsing_data - for
  return 0;
}

int SyncCylinderParsingSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& sync_cylinder_parse{data_msg->MutableSyncCylinderParsing()};

  sync_cylinder_parse.num_cylinder_parsing_data =
      proto_msg->cylinder_parsing_data_size();
  for (std::uint32_t cpd_idx = 0;
       cpd_idx < sync_cylinder_parse.num_cylinder_parsing_data; cpd_idx++) {
    auto& cpd_data{sync_cylinder_parse.cylinder_parsing_data.at(cpd_idx)};
    auto& cpd_proto{proto_msg->cylinder_parsing_data(cpd_idx)};
    cpd_data.camera_idx = static_cast<std::uint8_t>(cpd_proto.camera_idx());
    cpd_data.scale = cpd_proto.scale();

    if (cpd_proto.has_feature_map()) {
      auto& feature_map_data{cpd_data.feature_map};
      auto& feature_map_proto{cpd_proto.feature_map()};
      auto& value{feature_map_proto.data()};
      if (value.length() > kMaxCylParsingDataSize) {
        throw OutOfResourceException("sync_cylinder_parsing \
            feature_map_data->data info size too long");
      }
      feature_map_data.width = feature_map_proto.width();
      feature_map_data.height = feature_map_proto.height();
      feature_map_data.channel = feature_map_proto.channel();
      RleUtils::Decompress(reinterpret_cast<std::uint8_t const*>(value.c_str()), value.length(), 
        static_cast<std::uint8_t*>(feature_map_data.data), feature_map_data.width * feature_map_data.height);
    }  // end feature_map - for

    if (cpd_proto.has_roi_output()) {
      auto& roi_data{cpd_data.roi_output};
      auto& roi_proto{cpd_proto.roi_output()};
      roi_data.bottom = roi_proto.bottom();
      roi_data.right = roi_proto.right();
      roi_data.left = roi_proto.left();
      roi_data.top = roi_proto.top();
    }  // end roi_output
  }    // end cylinder_parsing_data - for
  return 0;
}

}  // namespace autodrive