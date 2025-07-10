//

#include "interface/parking/serializer/lane_multitask_serializer.h"

#include <utils/exception.h>
#include "utils/rle_utils.h"

#include "interface/parking/lane_multitask_data.h"
#include "protocol/lane_multitask.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int LaneMultitaskSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                       message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  {
    auto& tran_point_data{data_msg->GetTransitionPoint()};
    workflow::proto::LaneTransitionPoint* tran_point_proto =
        proto_msg->mutable_lane_transition_point();
    for (std::uint8_t idx = 0; idx < tran_point_data.num_transition_point;
         idx++) {
      auto& point_data{tran_point_data.transition_points.at(idx)};
      workflow::proto::TransitionPoints* point_proto =
          tran_point_proto->add_transition_points();
      point_proto->set_long_position(point_data.long_position);
      point_proto->set_lat_position(point_data.lat_position);
      point_proto->set_prob(point_data.prob);
      point_proto->set_type(
          static_cast<workflow::proto::TransitionType>(point_data.type));
    }  // end for - transition_points
  }    // end lane_transition_point_

  {
    auto& parse_image_data{data_msg->GetFeatureMap()};
    auto& parse_rle_data{data_msg->MutableRleFeatureMap()};
    for (size_t idx = 0; idx < data_msg->GetLaneParsingImageNum(); idx++) {
      auto& image_data{parse_image_data.at(idx)};
      const auto& rle_feature_map = parse_rle_data.at(idx);
      uint32_t num_rle_list_buf_f = rle_feature_map.num_rle_list_buf;
      uint32_t num_buf_data_f = rle_feature_map.num_data;
      RleList* rle_list_buf_f = rle_feature_map.rle_list_buf;
      std::vector<RleList> rle_list_buf_tmp;
      if(!rle_list_buf_f) {
        RleUtils::Compress(static_cast<std::uint8_t const*>(image_data.data), image_data.width, image_data.height,
                           rle_list_buf_tmp, &num_rle_list_buf_f, &num_buf_data_f);
        rle_list_buf_f = rle_list_buf_tmp.data();
      }
      LaneMultitaskData::LaneParsingImageData tmp_compressed_feature_map;
      RleUtils::WriteToBuffer(rle_list_buf_f, num_rle_list_buf_f,
          image_data.width, image_data.height, static_cast<std::uint8_t*>(tmp_compressed_feature_map.data));

      workflow::proto::LaneParsingImageData* image_proto =
          proto_msg->add_lane_parsing_image();
      image_proto->set_data((char*)tmp_compressed_feature_map.data, num_buf_data_f);
      image_proto->set_width(image_data.width);
      image_proto->set_height(image_data.height);
      image_proto->set_channel(image_data.channel);
    }  // end for
  }    // end lane_parsing_image_

  {
    proto_msg->set_camera_id(data_msg->GetCameraId());
  }    // end camera_id_

  {
    auto& lane_contours_data{data_msg->GetLaneContours()};
    for (size_t idx = 0; idx < data_msg->GetLaneContourNum(); idx++) {
      auto& contour_data{lane_contours_data.at(idx)};
      workflow::proto::LaneContour* contour_proto =
          proto_msg->add_lane_contours();
      {
        for (size_t pt_idx = 0; pt_idx < contour_data.num_basic_pts; pt_idx++) {
          auto& basic_pt{contour_data.basic_pts.at(pt_idx)};
          auto* basic_pt_proto = contour_proto->add_basic_pts();
          basic_pt_proto->mutable_pt()->set_x(basic_pt.pt.x);
          basic_pt_proto->mutable_pt()->set_y(basic_pt.pt.y);
          basic_pt_proto->mutable_pt_rle()->set_x(basic_pt.pt_rle.x);
          basic_pt_proto->mutable_pt_rle()->set_y(basic_pt.pt_rle.y);
          basic_pt_proto->mutable_pt_oriimg()->set_x(basic_pt.pt_oriimg.x);
          basic_pt_proto->mutable_pt_oriimg()->set_y(basic_pt.pt_oriimg.y);
          basic_pt_proto->mutable_pt_img()->set_x(basic_pt.pt_img.x);
          basic_pt_proto->mutable_pt_img()->set_y(basic_pt.pt_img.y);
          basic_pt_proto->set_label(basic_pt.label);
          basic_pt_proto->set_conf(basic_pt.conf);
        }
      }  // basic_pts

      {
        for (size_t pt_idx = 0; pt_idx < contour_data.num_extra_pts; pt_idx++) {
          auto& extra_pt{contour_data.extra_pts.at(pt_idx)};
          auto* extra_pt_proto = contour_proto->add_extra_pts();
          extra_pt_proto->set_xstart(extra_pt.xstart);
          extra_pt_proto->set_xend(extra_pt.xend);
          extra_pt_proto->set_y_parsing(extra_pt.y_parsing);
          extra_pt_proto->set_gray(extra_pt.gray);
          extra_pt_proto->mutable_pt_rle_c()->set_x(extra_pt.pt_rle_c.x);
          extra_pt_proto->mutable_pt_rle_c()->set_y(extra_pt.pt_rle_c.y);
          extra_pt_proto->mutable_width_info()->set_x(extra_pt.width_info.x);
          extra_pt_proto->mutable_width_info()->set_y(extra_pt.width_info.y);
          extra_pt_proto->set_flag_dash_endpt(extra_pt.flag_dash_endpt);
          extra_pt_proto->set_horizontal(extra_pt.horizontal);
        }
      }  // extra_pts
    }    // end for
  }      // end lane_contours_

  {
    proto_msg->mutable_roi()->set_bottom(data_msg->GetRoi().bottom);
    proto_msg->mutable_roi()->set_top(data_msg->GetRoi().top);
    proto_msg->mutable_roi()->set_left(data_msg->GetRoi().left);
    proto_msg->mutable_roi()->set_right(data_msg->GetRoi().right);
  }  //  roi_

  return 0;
}

int LaneMultitaskSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);

  {
    auto& tran_point_data{data_msg->MutableTransitionPoint()};
    auto& tran_point_proto{proto_msg->lane_transition_point()};
    tran_point_data.num_transition_point =
        static_cast<std::uint8_t>(tran_point_proto.transition_points_size());
    for (std::uint16_t idx = 0; idx < tran_point_data.num_transition_point;
         idx++) {
      auto& point_data{tran_point_data.transition_points.at(idx)};
      auto& point_proto{tran_point_proto.transition_points(idx)};
      point_data.long_position = point_proto.long_position();
      point_data.lat_position = point_proto.lat_position();
      point_data.prob = point_proto.prob();
      point_data.type =
          static_cast<RoadInfoData::TransitionType>(point_proto.type());
    }  // end for
  }    // end lane_transition_point_

  {
    auto& parse_image_data{data_msg->MutableFeatureMap()};
    data_msg->SetLaneParsingImageNum(proto_msg->lane_parsing_image_size());
    for (std::int32_t idx = 0; idx < proto_msg->lane_parsing_image_size();
         idx++) {
      auto& image_data{parse_image_data.at(idx)};
      auto& image_proto{proto_msg->lane_parsing_image(idx)};
      auto& value{image_proto.data()};

      if (value.length() >
          LaneMultitaskData::kImageWidth * LaneMultitaskData::kImageHeight) {
        throw OutOfResourceException(
            "lane_multitask image_data->data info size too long");
      }

      // memcpy(image_data.data, value.c_str(), value.length());
      image_data.width = image_proto.width();
      image_data.height = image_proto.height();
      image_data.channel = image_proto.channel();
      RleUtils::Decompress(reinterpret_cast<std::uint8_t const*>(value.c_str()), value.length(), 
        static_cast<std::uint8_t*>(image_data.data), image_data.width * image_data.height);
    }  // end for
  }    // end lane_parsing_image_

  {
    data_msg->SetCameraId(proto_msg->camera_id());
  }    // end camera_id_

  {
    data_msg->SetLaneContourNum(proto_msg->lane_contours_size());

    for (size_t contour_idx = 0;
         contour_idx < static_cast<uint32_t>(proto_msg->lane_contours_size());
         contour_idx++) {
      auto& lane_contour_data{data_msg->MutableLaneContours().at(contour_idx)};
      auto& lane_contour_proto = proto_msg->lane_contours(contour_idx);

      {
        lane_contour_data.num_basic_pts = lane_contour_proto.basic_pts_size();
        if (lane_contour_data.num_basic_pts >
            LaneMultitaskData::kMaxBasicPtsNum) {
          throw OutOfResourceException(
              "lane_multitask lane_contour num_basic_pts too large");
        }

        for (size_t basic_idx = 0; basic_idx < lane_contour_data.num_basic_pts;
             ++basic_idx) {
          auto& basic_proto = lane_contour_proto.basic_pts(basic_idx);
          auto& basic_data = lane_contour_data.basic_pts[basic_idx];
          basic_data.pt.x = basic_proto.pt().x();
          basic_data.pt.y = basic_proto.pt().y();
          basic_data.pt_rle.x = basic_proto.pt_rle().x();
          basic_data.pt_rle.y = basic_proto.pt_rle().y();
          basic_data.pt_oriimg.x = basic_proto.pt_oriimg().x();
          basic_data.pt_oriimg.y = basic_proto.pt_oriimg().y();
          basic_data.pt_img.x = basic_proto.pt_img().x();
          basic_data.pt_img.y = basic_proto.pt_img().y();
          basic_data.label = basic_proto.label();
          basic_data.conf = basic_proto.conf();
        }
      }  // basic_pts

      {
        lane_contour_data.num_extra_pts = lane_contour_proto.extra_pts_size();
        if (lane_contour_data.num_extra_pts >
            LaneMultitaskData::kMaxExtraPtsNum) {
          throw OutOfResourceException(
              "lane_multitask lane_contour num_extra_pts too large");
        }

        for (size_t extra_idx = 0; extra_idx < lane_contour_data.num_extra_pts;
             ++extra_idx) {
          auto& extra_proto = lane_contour_proto.extra_pts(extra_idx);
          auto& extra_data = lane_contour_data.extra_pts[extra_idx];
          extra_data.xstart = extra_proto.xstart();
          extra_data.xend = extra_proto.xend();
          extra_data.y_parsing = extra_proto.y_parsing();
          extra_data.gray = extra_proto.gray();
          extra_data.pt_rle_c.x = extra_proto.pt_rle_c().x();
          extra_data.pt_rle_c.y = extra_proto.pt_rle_c().y();
          extra_data.width_info.x = extra_proto.width_info().x();
          extra_data.width_info.y = extra_proto.width_info().y();
          extra_data.flag_dash_endpt = extra_proto.flag_dash_endpt();
          extra_data.horizontal = extra_proto.horizontal();
        }
      }  // extra_pts
    }
  }  // end lane_contours_

  return 0;
}

}  // namespace autodrive