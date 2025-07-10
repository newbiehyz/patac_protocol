//
#include "interface/parking/serializer/road_info_serializer.h"

#include <utils/exception.h>

#include "interface/parking/road_info_data.h"
#include "protocol/road_info.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int RoadInfoSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                  message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& road_data{data_msg->GetRoadInfo()};

  {
    auto& border_data{road_data.road_borders};
    workflow::proto::RoadBorders* border_proto =
        proto_msg->mutable_road_borders();
    for (std::int32_t b_idx = 0; b_idx < border_data.nums_borders; b_idx++) {
      auto& b_data{border_data.borders.at(b_idx)};
      workflow::proto::Border* b_proto = border_proto->add_borders();
      b_proto->set_border_side(
          static_cast<workflow::proto::RoadBorderSide>(b_data.border_side));
      b_proto->set_border_type(
          static_cast<workflow::proto::RoadBorderType>(b_data.border_type));
      b_proto->set_conf_level(
          static_cast<workflow::proto::RoadInfoLineSource>(b_data.conf_level));

      {
        auto& curve_line_data{b_data.curve};
        workflow::proto::RoadInfoCurveLine* curve_line_proto =
            b_proto->mutable_curve();
        curve_line_proto->set_start(curve_line_data.start);
        curve_line_proto->set_end(curve_line_data.end);
        curve_line_proto->set_rmse(curve_line_data.rmse);
        curve_line_proto->set_type(
            static_cast<workflow::proto::RoadInfoCurveType>(
                curve_line_data.type));

        for (std::uint8_t p_idx = 0; p_idx < curve_line_data.num_points;
             p_idx++) {
          auto& point_data{curve_line_data.points.at(p_idx)};
          workflow::proto::Vector3d* ponit_proto =
              curve_line_proto->add_points();
          ponit_proto->set_x(point_data.x);
          ponit_proto->set_y(point_data.y);
          ponit_proto->set_z(point_data.z);
        }  // end points

        for (std::uint8_t t_idx = 0; t_idx < curve_line_data.num_t; t_idx++) {
          curve_line_proto->add_t(curve_line_data.t.at(t_idx));
        }

        curve_line_proto->set_t_max(curve_line_data.t_max);

        curve_line_proto->mutable_start_pt()->set_x(curve_line_data.start_pt.x);
        curve_line_proto->mutable_start_pt()->set_y(curve_line_data.start_pt.y);
        curve_line_proto->mutable_start_pt()->set_z(curve_line_data.start_pt.z);

        for (std::uint8_t x_idx = 0; x_idx < curve_line_data.num_x_coeff;
             x_idx++) {
          curve_line_proto->add_x_coeff(curve_line_data.x_coeff.at(x_idx));
        }

        for (std::uint8_t y_idx = 0; y_idx < curve_line_data.num_y_coeff;
             y_idx++) {
          curve_line_proto->add_y_coeff(curve_line_data.y_coeff.at(y_idx));
        }
      }  // end curve

      b_proto->set_conf(b_data.conf);
      b_proto->set_border_height(b_data.border_height);
    }  // end for
  }    // end road_borders

  {
    auto& lane_lines_data{road_data.lane_lines};
    workflow::proto::RoadInfoLines* lane_lines_proto =
        proto_msg->mutable_lane_lines();
    {
      auto& camera_data{lane_lines_data.camera_matrix};
      workflow::proto::perception_base::CameraMatrix* camera_proto =
          lane_lines_proto->mutable_camera_matrix();

      for (size_t gi_idx = 0; gi_idx < camera_data.mat_gnd2img.size();
           gi_idx++) {
        camera_proto->add_mat_gnd2img(camera_data.mat_gnd2img.at(gi_idx));
      }

      for (size_t ig_idx = 0; ig_idx < camera_data.mat_img2gnd.size();
           ig_idx++) {
        camera_proto->add_mat_img2gnd(camera_data.mat_img2gnd.at(ig_idx));
      }

      for (size_t vgi_idx = 0; vgi_idx < camera_data.mat_vcsgnd2img.size();
           vgi_idx++) {
        camera_proto->add_mat_vcsgnd2img(camera_data.mat_vcsgnd2img.at(vgi_idx));
      }

      for (size_t ivg_idx = 0; ivg_idx < camera_data.mat_img2vcsgnd.size();
           ivg_idx++) {
        camera_proto->add_mat_img2vcsgnd(camera_data.mat_img2vcsgnd.at(ivg_idx));
      }

      for (size_t li_idx = 0; li_idx < camera_data.mat_local2img.size();
           li_idx++) {
        camera_proto->add_mat_local2img(camera_data.mat_local2img.at(li_idx));
      }

      for (size_t il_idx = 0; il_idx < camera_data.mat_img2local.size();
           il_idx++) {
        camera_proto->add_mat_img2local(camera_data.mat_img2local.at(il_idx));
      }

      for (size_t vi_idx = 0; vi_idx < camera_data.mat_vcs2img.size();
           vi_idx++) {
        camera_proto->add_mat_vcs2img(camera_data.mat_vcs2img.at(vi_idx));
      }
    }  // end camera_matrix

    for (std::int32_t l_idx = 0; l_idx < lane_lines_data.num_lines; l_idx++) {
      auto& line_data{lane_lines_data.lines.at(l_idx)};
      workflow::proto::RoadInfoLine* line_proto = lane_lines_proto->add_lines();

      line_proto->set_id(line_data.id);
      line_proto->set_str_id(line_data.str_id.data(), line_data.num_str_id);

      for (std::int32_t cs_idx = 0; cs_idx < line_data.num_curve_segs;
           cs_idx++) {
        auto& curve_line_data{line_data.curve_segs.at(cs_idx)};
        workflow::proto::RoadInfoCurveLine* curve_line_proto =
            line_proto->add_curve_segs();
        curve_line_proto->set_start(curve_line_data.start);
        curve_line_proto->set_end(curve_line_data.end);
        curve_line_proto->set_rmse(curve_line_data.rmse);
        curve_line_proto->set_type(
            static_cast<workflow::proto::RoadInfoCurveType>(
                curve_line_data.type));

        for (std::uint8_t p_idx = 0; p_idx < curve_line_data.num_points;
             p_idx++) {
          auto* point = curve_line_proto->add_points();
          point->set_x(curve_line_data.points.at(p_idx).x);
          point->set_y(curve_line_data.points.at(p_idx).y);
          point->set_z(curve_line_data.points.at(p_idx).z);
        }

        for (std::uint8_t t_idx = 0; t_idx < curve_line_data.num_t; t_idx++) {
          curve_line_proto->add_t(curve_line_data.t.at(t_idx));
        }
        curve_line_proto->set_t_max(curve_line_data.t_max);

        curve_line_proto->mutable_start_pt()->set_x(curve_line_data.start_pt.x);
        curve_line_proto->mutable_start_pt()->set_y(curve_line_data.start_pt.y);
        curve_line_proto->mutable_start_pt()->set_z(curve_line_data.start_pt.z);

        for (std::uint8_t x_idx = 0; x_idx < curve_line_data.num_x_coeff;
             x_idx++) {
          curve_line_proto->add_x_coeff(curve_line_data.x_coeff.at(x_idx));
        }

        for (std::uint8_t y_idx = 0; y_idx < curve_line_data.num_y_coeff;
             y_idx++) {
          curve_line_proto->add_y_coeff(curve_line_data.y_coeff.at(y_idx));
        }
      }  // end curve_segs

      line_proto->set_start(line_data.start);
      line_proto->set_end(line_data.end);
      line_proto->set_conf(line_data.conf);
      line_proto->set_width(line_data.width);

      line_proto->set_position(
          static_cast<workflow::proto::RoadInfoLinePosition>(
              line_data.position));
      line_proto->set_conf_level(
          static_cast<workflow::proto::RoadInfoLineSource>(
              line_data.conf_level));
      line_proto->set_line_color(
          static_cast<workflow::proto::RoadInfoLineColor>(
              line_data.line_color));
      line_proto->set_type(
          static_cast<workflow::proto::RoadInfoLineType>(line_data.type));
      line_proto->set_dlm_type(
          static_cast<workflow::proto::DlmType>(line_data.dlm_type));
      line_proto->set_decel_type(
          static_cast<workflow::proto::DecelType>(line_data.decel_type));

      line_proto->set_road_prediction(line_data.road_prediction);
      line_proto->set_triggered_model(
          static_cast<workflow::proto::TriggerModel>(line_data.triggered_model));
    }  // end lines - for

    lane_lines_proto->set_src_time_stamp(lane_lines_data.src_time_stamp);
    lane_lines_proto->set_asyn_time_stamp(lane_lines_data.asyn_time_stamp);
    lane_lines_proto->set_ambiguous_line_pattern_left(
        lane_lines_data.ambiguous_line_pattern_left);
    lane_lines_proto->set_ambiguous_line_pattern_right(
        lane_lines_data.ambiguous_line_pattern_right);
    lane_lines_proto->set_construction_area(lane_lines_data.construction_area);
    lane_lines_proto->set_lane_change(
        static_cast<workflow::proto::LaneChange>(lane_lines_data.lane_change));
    lane_lines_proto->set_driving_side(
        static_cast<workflow::proto::RoadInfoLines_DrivingSide>(
            lane_lines_data.driving_side));
    lane_lines_proto->set_host_width(lane_lines_data.host_width);

    for (std::int32_t tp_idx = 0;
         tp_idx < lane_lines_data.num_transition_points; tp_idx++) {
      auto& tpoint_data{lane_lines_data.transition_points.at(tp_idx)};
      workflow::proto::TransitionPoints* tpoint_proto =
          lane_lines_proto->add_transition_points();
      tpoint_proto->set_long_position(tpoint_data.long_position);
      tpoint_proto->set_lat_position(tpoint_data.lat_position);
      tpoint_proto->set_prob(tpoint_data.prob);
      tpoint_proto->set_type(
          static_cast<workflow::proto::TransitionType>(tpoint_data.type));
    }  // end transition_points

    {
      auto& hj_data{lane_lines_data.highway_junction_info};
      workflow::proto::HighWayJunctionInfo* hj_proto =
          lane_lines_proto->mutable_highway_junction_info();
      hj_proto->set_highway_entry_left(hj_data.highway_exit_left);
      hj_proto->set_highway_exit_right(hj_data.highway_exit_right);
      hj_proto->set_highway_entry_left(hj_data.highway_entry_left);
      hj_proto->set_highway_entry_right(hj_data.highway_entry_right);
    }  // end highway_junction_info

    {
      auto& num_lane_data{lane_lines_data.number_of_lanes};
      workflow::proto::NumberOfLanes* num_lane_proto =
          lane_lines_proto->mutable_number_of_lanes();
      num_lane_proto->set_range_0m(num_lane_data.range_0m);
      num_lane_proto->set_range_35m(num_lane_data.range_35m);
    }  // end number_of_lanes
  }    // end lane_lines

  {
    auto& stop_lines_data{road_data.stop_lines};
    workflow::proto::StopLines* stop_lines_proto =
        proto_msg->mutable_stop_lines();
    stop_lines_proto->set_lateral_distance(stop_lines_data.lateral_distance);
    stop_lines_proto->set_longitudinal_distance(
        stop_lines_data.longitudinal_distance);
    stop_lines_proto->set_angle(stop_lines_data.angle);
    stop_lines_proto->set_confidence(stop_lines_data.confidence);
    stop_lines_proto->set_id(stop_lines_data.id);

    stop_lines_proto->set_stop_line_status(
        static_cast<workflow::proto::StopLineStatus>(
            stop_lines_data.stop_line_status));
    stop_lines_proto->set_stop_line_type(
        static_cast<workflow::proto::StopLineType>(
            stop_lines_data.stop_line_type));
    stop_lines_proto->set_stop_line_color(
        static_cast<workflow::proto::StopLineColorType>(
            stop_lines_data.stop_line_color));
    stop_lines_proto->set_is_relevant(stop_lines_data.is_relevant);
  }  // end stop_lines

  {
    auto& cross_data{road_data.crossings};
    workflow::proto::Crossings* cross_proto = proto_msg->mutable_crossings();
    cross_proto->set_lateral_position(cross_data.lateral_position);
    cross_proto->set_longitudinal_position(cross_data.longitudinal_position);
    cross_proto->set_confidence(cross_data.confidence);
    cross_proto->set_id(cross_data.id);
    cross_proto->set_crossings_type(
        static_cast<workflow::proto::CrossingsType>(cross_data.crossings_type));
  }  // end crossings

  {
    proto_msg->set_camera_id(data_msg->GetCameraId());
  }  // end camera id

  return 0;
}

int RoadInfoSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& road_data{data_msg->MutableRoadInfo()};

  if (proto_msg->has_road_borders()) {
    auto& borders_data{road_data.road_borders};
    auto& borders_proto{proto_msg->road_borders()};

    borders_data.nums_borders = borders_proto.borders_size();
    for (std::int32_t b_idx = 0; b_idx < borders_data.nums_borders; b_idx++) {
      auto& border_data{borders_data.borders.at(b_idx)};
      auto& border_proto{borders_proto.borders(b_idx)};
      border_data.border_side =
          static_cast<RoadInfoData::RoadBorderSide>(border_proto.border_side());
      border_data.border_type =
          static_cast<RoadInfoData::RoadBorderType>(border_proto.border_type());
      border_data.conf_level = static_cast<RoadInfoData::RoadInfoLineSource>(
          border_proto.conf_level());

      if (border_proto.has_curve()) {
        auto& curve_data{border_data.curve};
        auto& curve_proto{border_proto.curve()};

        curve_data.start = curve_proto.start();
        curve_data.end = curve_proto.end();
        curve_data.rmse = curve_proto.rmse();
        curve_data.type =
            static_cast<RoadInfoData::RoadInfoCurveType>(curve_proto.type());

        curve_data.num_points = curve_proto.points_size();
        for (std::uint8_t p_idx = 0; p_idx < curve_data.num_points; p_idx++) {
          auto& point_data{curve_data.points.at(p_idx)};
          auto& point_proto{curve_proto.points(p_idx)};
          point_data.x = point_proto.x();
          point_data.y = point_proto.y();
          point_data.z = point_proto.z();
        }  // end points

        curve_data.num_t = curve_proto.t_size();
        for (std::uint8_t t_idx = 0; t_idx < curve_data.num_t; t_idx++) {
          curve_data.t.at(t_idx) = curve_proto.t(t_idx);
        }  // end t

        curve_data.t_max = curve_proto.t_max();
        curve_data.start_pt.x = curve_proto.start_pt().x();
        curve_data.start_pt.y = curve_proto.start_pt().y();
        curve_data.start_pt.z = curve_proto.start_pt().z();

        curve_data.num_x_coeff = curve_proto.x_coeff_size();
        for (std::uint8_t x_idx = 0; x_idx < curve_data.num_x_coeff; x_idx++) {
          curve_data.x_coeff.at(x_idx) = curve_proto.x_coeff(x_idx);
        }  // end x_coeff

        curve_data.num_y_coeff = curve_proto.y_coeff_size();
        for (std::uint8_t y_idx = 0; y_idx < curve_data.num_y_coeff; y_idx++) {
          curve_data.y_coeff.at(y_idx) = curve_proto.y_coeff(y_idx);
        }  // end y_coeff
      }    // end curve

      border_data.conf = border_proto.conf();
      border_data.border_height = border_proto.border_height();
    }  // end borders - for
  }    // end road_borders

  if (proto_msg->has_lane_lines()) {
    auto& lane_line_data{road_data.lane_lines};
    auto& lane_line_proto{proto_msg->lane_lines()};

    if (lane_line_proto.has_camera_matrix()) {
      auto& camear_data{lane_line_data.camera_matrix};
      auto& camera_proto{lane_line_proto.camera_matrix()};

      for (std::int32_t gi_idx = 0; gi_idx < camera_proto.mat_gnd2img_size();
           gi_idx++) {
        camear_data.mat_gnd2img.at(gi_idx) = camera_proto.mat_gnd2img(gi_idx);
      }

      for (std::int32_t ig_idx = 0; ig_idx < camera_proto.mat_img2gnd_size();
           ig_idx++) {
        camear_data.mat_img2gnd.at(ig_idx) = camera_proto.mat_img2gnd(ig_idx);
      }

      for (std::int32_t vgi_idx = 0;
           vgi_idx < camera_proto.mat_vcsgnd2img_size(); vgi_idx++) {
        camear_data.mat_vcsgnd2img.at(vgi_idx) =
            camera_proto.mat_vcsgnd2img(vgi_idx);
      }

      for (std::int32_t ivg_idx = 0;
           ivg_idx < camera_proto.mat_img2vcsgnd_size(); ivg_idx++) {
        camear_data.mat_img2vcsgnd.at(ivg_idx) =
            camera_proto.mat_img2vcsgnd(ivg_idx);
      }

      for (std::int32_t li_idx = 0; li_idx < camera_proto.mat_local2img_size();
           li_idx++) {
        camear_data.mat_local2img.at(li_idx) =
            camera_proto.mat_local2img(li_idx);
      }

      for (std::int32_t il_idx = 0; il_idx < camera_proto.mat_img2local_size();
           il_idx++) {
        camear_data.mat_img2local.at(il_idx) =
            camera_proto.mat_img2local(il_idx);
      }

      for (std::int32_t vi_idx = 0; vi_idx < camera_proto.mat_vcs2img_size();
           vi_idx++) {
        camear_data.mat_vcs2img.at(vi_idx) = camera_proto.mat_vcs2img(vi_idx);
      }
    }  // camera_matrix

    lane_line_data.num_lines = lane_line_proto.lines_size();
    for (std::int32_t l_idx = 0; l_idx < lane_line_data.num_lines; l_idx++) {
      auto& line_data{lane_line_data.lines.at(l_idx)};
      auto& line_proto{lane_line_proto.lines(l_idx)};
      line_data.id = line_proto.id();
      auto& str_value{line_proto.str_id()};
      memcpy(line_data.str_id.data(), str_value.c_str(), str_value.length());

      line_data.num_curve_segs = line_proto.curve_segs_size();
      for (std::int32_t cs_idx = 0; cs_idx < line_data.num_curve_segs;
           cs_idx++) {
        auto& curve_data{line_data.curve_segs.at(cs_idx)};
        auto& curve_proto{line_proto.curve_segs(cs_idx)};
        curve_data.start = curve_proto.start();
        curve_data.end = curve_proto.end();
        curve_data.rmse = curve_proto.rmse();
        curve_data.type =
            static_cast<RoadInfoData::RoadInfoCurveType>(curve_proto.type());

        curve_data.num_points = curve_proto.points_size();
        for (std::uint8_t p_idx = 0; p_idx < curve_data.num_points; p_idx++) {
          auto& point_data{curve_data.points.at(p_idx)};
          auto& point_proto{curve_proto.points(p_idx)};
          point_data.x = point_proto.x();
          point_data.y = point_proto.y();
          point_data.z = point_proto.z();
        }

        curve_data.t_max = curve_proto.t_max();
        curve_data.start_pt.x = curve_proto.start_pt().x();
        curve_data.start_pt.y = curve_proto.start_pt().y();
        curve_data.start_pt.z = curve_proto.start_pt().z();

        curve_data.num_x_coeff = curve_proto.x_coeff_size();
        for (std::uint8_t x_idx = 0; x_idx < curve_data.num_x_coeff; x_idx++) {
          curve_data.x_coeff.at(x_idx) = curve_proto.x_coeff(x_idx);
        }

        curve_data.num_y_coeff = curve_proto.y_coeff_size();
        for (std::uint8_t y_idx = 0; y_idx < curve_data.num_y_coeff; y_idx++) {
          curve_data.y_coeff.at(y_idx) = curve_proto.y_coeff(y_idx);
        }
      }  // end curve_segs

      line_data.life_time = line_proto.life_time();
      line_data.start = line_proto.start();
      line_data.end = line_proto.end();
      line_data.conf = line_proto.conf();
      line_data.width = line_proto.width();

      line_data.position = static_cast<RoadInfoData::RoadInfoLinePosition>(
          line_proto.position());
      line_data.conf_level = static_cast<RoadInfoData::RoadInfoLineSource>(
          line_proto.conf_level());
      line_data.line_color =
          static_cast<RoadInfoData::RoadInfoLineColor>(line_proto.line_color());
      line_data.type =
          static_cast<RoadInfoData::RoadInfoLineType>(line_proto.type());
      line_data.dlm_type =
          static_cast<RoadInfoData::DlmType>(line_proto.dlm_type());
      line_data.decel_type =
          static_cast<RoadInfoData::DecelType>(line_proto.decel_type());
      line_data.road_prediction = line_proto.road_prediction();
      line_data.triggered_model =
          static_cast<RoadInfoData::TriggerModel>(line_proto.triggered_model());
    }  // end lines

    lane_line_data.src_time_stamp = lane_line_proto.src_time_stamp();
    lane_line_data.asyn_time_stamp = lane_line_proto.asyn_time_stamp();
    lane_line_data.ambiguous_line_pattern_left =
        lane_line_proto.ambiguous_line_pattern_left();
    lane_line_data.ambiguous_line_pattern_right =
        lane_line_proto.ambiguous_line_pattern_right();
    lane_line_data.construction_area = lane_line_proto.construction_area();
    lane_line_data.lane_change =
        static_cast<RoadInfoData::LaneChange>(lane_line_proto.lane_change());
    lane_line_data.driving_side =
        static_cast<RoadInfoData::DrivingSide>(lane_line_proto.driving_side());

    lane_line_data.host_width = lane_line_proto.host_width();

    lane_line_data.num_transition_points = lane_line_proto.transition_points_size();
    for (std::int32_t tp_idx = 0; tp_idx < lane_line_data.num_transition_points; tp_idx++) {
      auto& tpoint_data{lane_line_data.transition_points.at(tp_idx)};
      auto& tpoint_proto{lane_line_proto.transition_points(tp_idx)};
      tpoint_data.long_position = tpoint_proto.long_position();
      tpoint_data.lat_position = tpoint_proto.lat_position();
      tpoint_data.prob = tpoint_proto.prob();
      tpoint_data.type =
          static_cast<RoadInfoData::TransitionType>(tpoint_proto.type());
    }  // end transition_points

    if (lane_line_proto.has_highway_junction_info()) {
      auto& hj_data{lane_line_data.highway_junction_info};
      auto& hj_proto{lane_line_proto.highway_junction_info()};
      hj_data.highway_exit_left = hj_proto.highway_exit_left();
      hj_data.highway_exit_right = hj_proto.highway_exit_right();
      hj_data.highway_entry_left = hj_proto.highway_entry_left();
      hj_data.highway_entry_right = hj_proto.highway_entry_right();
    }  // end highway_junction_info

    if (lane_line_proto.has_number_of_lanes()) {
      auto& num_lanes_data{lane_line_data.number_of_lanes};
      auto& num_lanes_proto{lane_line_proto.number_of_lanes()};
      num_lanes_data.range_0m = num_lanes_proto.range_0m();
      num_lanes_data.range_35m = num_lanes_proto.range_35m();
    }  // end number_of_lanes
  }    // end lane_lines

  if (proto_msg->has_stop_lines()) {
    auto& stop_line_data{road_data.stop_lines};
    auto& stop_line_proto{proto_msg->stop_lines()};
    stop_line_data.lateral_distance = stop_line_proto.lateral_distance();
    stop_line_data.longitudinal_distance =
        stop_line_proto.longitudinal_distance();
    stop_line_data.angle = stop_line_proto.angle();
    stop_line_data.confidence = stop_line_proto.confidence();
    stop_line_data.id = stop_line_proto.id();

    stop_line_data.stop_line_status = static_cast<RoadInfoData::StopLineStatus>(
        stop_line_proto.stop_line_status());
    stop_line_data.stop_line_type = static_cast<RoadInfoData::StopLineType>(
        stop_line_proto.stop_line_type());
    stop_line_data.stop_line_color =
        static_cast<RoadInfoData::StopLineColorType>(
            stop_line_proto.stop_line_color());
    stop_line_data.is_relevant = stop_line_proto.is_relevant();
  }  // end stop_lines

  if (proto_msg->has_crossings()) {
    auto& cross_data{road_data.crossings};
    auto& cross_proto{proto_msg->crossings()};
    cross_data.lateral_position = cross_proto.lateral_position();
    cross_data.longitudinal_position = cross_proto.longitudinal_position();
    cross_data.confidence = cross_proto.confidence();
    cross_data.id = cross_proto.id();
    cross_data.crossings_type =
        static_cast<RoadInfoData::CrossingsType>(cross_proto.crossings_type());
  }  // end crossings

  {
    data_msg->SetCameraId(proto_msg->camera_id());
  }  // end camera_id_

  return 0;
}

}  // namespace autodrive