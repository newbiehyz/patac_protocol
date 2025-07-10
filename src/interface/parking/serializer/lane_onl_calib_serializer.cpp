//

#include "interface/parking/serializer/lane_onl_calib_serializer.h"

#include <utils/exception.h>
#include "utils/rle_utils.h"

#include "interface/parking/lane_onl_calib_data.h"
#include "protocol/lane_onl_calib.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int LaneOnlCalibSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                      message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  {
    const auto& camera_lane_data = data_msg->GetCameraLane();
    for (uint32_t idx = 0; idx < data_msg->GetCameraNum(); idx++) {
      const auto& lane_data = camera_lane_data.at(idx);
      auto* camera_lane_msg = proto_msg->add_camera_lane();

      camera_lane_msg->set_camera_id(lane_data.camera_id);
      for (uint32_t parsing_idx = 0;
           parsing_idx < lane_data.num_lane_parsing_image; ++parsing_idx) {
        const auto& parsing_data = lane_data.lane_parsing_image[parsing_idx];
        uint32_t num_rle_list_buf_f{0}, num_buf_data_f{0};
        std::vector<RleList> rle_list_buf_tmp;
        RleUtils::Compress(static_cast<std::uint8_t const*>(parsing_data.data), parsing_data.width, parsing_data.height,
                           rle_list_buf_tmp, &num_rle_list_buf_f, &num_buf_data_f);
        LaneOnlCalibData::LaneParsingImageData tmp_compressed_feature_map;
        RleUtils::WriteToBuffer(rle_list_buf_tmp.data(), num_rle_list_buf_f,
            parsing_data.width, parsing_data.height, static_cast<std::uint8_t*>(tmp_compressed_feature_map.data));

        auto* parsing_msg = camera_lane_msg->add_lane_parsing_image();
        parsing_msg->set_data((char*)tmp_compressed_feature_map.data, num_buf_data_f);
        parsing_msg->set_width(parsing_data.width);
        parsing_msg->set_height(parsing_data.height);
        parsing_msg->set_channel(parsing_data.channel);
      }

      camera_lane_msg->mutable_parsing_roi()->set_bottom(
          lane_data.parsing_roi.bottom);
      camera_lane_msg->mutable_parsing_roi()->set_top(
          lane_data.parsing_roi.top);
      camera_lane_msg->mutable_parsing_roi()->set_left(
          lane_data.parsing_roi.left);
      camera_lane_msg->mutable_parsing_roi()->set_right(
          lane_data.parsing_roi.right);
    }
  }

  return 0;
}

int LaneOnlCalibSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                        message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  {
    if (static_cast<uint32_t>(proto_msg->camera_lane_size()) >
        LaneOnlCalibData::kMaxCameraNumber) {
      throw OutOfResourceException("lane_onl_calib camera num too long");
    }

    data_msg->SetCameraNum(proto_msg->camera_lane_size());

    for (uint32_t idx = 0; idx < data_msg->GetCameraNum(); idx++) {
      auto& lane_data{data_msg->MutableCameraLane().at(idx)};
      auto& lane_proto{proto_msg->camera_lane(idx)};

      lane_data.camera_id = lane_proto.camera_id();
      if (static_cast<uint32_t>(lane_proto.lane_parsing_image_size()) >
          LaneOnlCalibData::kMaxParsingSize) {
        throw OutOfResourceException(
            "lane_onl_calib parsing image num too long");
      }

      lane_data.num_lane_parsing_image = lane_proto.lane_parsing_image_size();

      for (uint32_t parsing_idx = 0;
           parsing_idx < lane_data.num_lane_parsing_image; ++parsing_idx) {
        auto& parsing_data{lane_data.lane_parsing_image.at(parsing_idx)};
        auto& parsing_proto{lane_proto.lane_parsing_image(parsing_idx)};

        auto& value{parsing_proto.data()};
        if (value.length() >
            LaneOnlCalibData::kImageWidth * LaneOnlCalibData::kImageHeight) {
          throw OutOfResourceException(
              "lane_onl_calib parsing_data->data info size too long");
        }
        parsing_data.width = parsing_proto.width();
        parsing_data.height = parsing_proto.height();
        parsing_data.channel = parsing_proto.channel();
        RleUtils::Decompress(reinterpret_cast<std::uint8_t const*>(value.c_str()), value.length(),
          static_cast<std::uint8_t*>(parsing_data.data), parsing_data.width * parsing_data.height);
      }

      lane_data.parsing_roi.bottom = lane_proto.parsing_roi().bottom();
      lane_data.parsing_roi.top = lane_proto.parsing_roi().top();
      lane_data.parsing_roi.left = lane_proto.parsing_roi().left();
      lane_data.parsing_roi.right = lane_proto.parsing_roi().right();

    }  // end lane_transition_point_
  }

  return 0;
}

}  // namespace autodrive