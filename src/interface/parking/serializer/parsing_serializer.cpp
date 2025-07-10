//
#include "interface/parking/serializer/parsing_serializer.h"

#include <utils/exception.h>

#include "interface/parking/parsing_data.h"
#include "protocol/parsing.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int ParsingSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                 message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& parsing_data{data_msg->GetParsing()};

  workflow::proto::ParsingImageData* free_space_proto =
      proto_msg->mutable_free_space();
  free_space_proto->set_data((char*)(parsing_data.free_space.data), 200704U);
  free_space_proto->set_width(parsing_data.free_space.width);
  free_space_proto->set_height(parsing_data.free_space.height);
  free_space_proto->set_channel(parsing_data.free_space.channel);

  workflow::proto::ParsingImageData* feat_map_proto =
      proto_msg->mutable_feature_map();
  feat_map_proto->set_data((char*)(parsing_data.feature_map.data), 200704U);
  feat_map_proto->set_width(parsing_data.feature_map.width);
  feat_map_proto->set_height(parsing_data.feature_map.height);
  feat_map_proto->set_channel(parsing_data.feature_map.channel);

  proto_msg->set_scale(parsing_data.scale);

  auto& roi_data{parsing_data.roi_output};
  proto_msg->mutable_roi_output()->set_left(roi_data.left);
  proto_msg->mutable_roi_output()->set_top(roi_data.top);
  proto_msg->mutable_roi_output()->set_right(roi_data.right);
  proto_msg->mutable_roi_output()->set_bottom(roi_data.bottom);

  workflow::proto::ParsingImage* image_proto = proto_msg->mutable_image();
  auto& image_data{parsing_data.image};
  image_proto->set_width(image_data.width);
  image_proto->set_height(image_data.height);
  image_proto->set_channel(image_data.channel);
  image_proto->set_send_mod(image_data.send_mod);
  image_proto->set_time_stamp(image_data.time_stamp);

  image_proto->set_format(image_data.format);
  image_proto->set_index(image_data.index);
  image_proto->set_count(image_data.count);
  image_proto->set_depth(image_data.depth);
  image_proto->set_align(image_data.align);

  image_proto->set_image_counter(image_data.image_counter);
  image_proto->set_perception_status(image_data.perception_status);
  image_proto->set_bypass_time_diff(image_data.bypass_time_diff);

  image_proto->set_compression_format(image_data.compression_format);
  image_proto->set_status(image_data.status);

  for (size_t r_idx = 0; r_idx < image_data.roi_map.size(); r_idx++) {
    workflow::proto::perception_base::RoiMap* roi_proro =
        image_proto->mutable_roi_map(r_idx);
    auto& roi_data{image_data.roi_map.at(r_idx)};
    roi_proro->mutable_model_roi()->set_left(roi_data.model_roi.left);
    roi_proro->mutable_model_roi()->set_top(roi_data.model_roi.top);
    roi_proro->mutable_model_roi()->set_right(roi_data.model_roi.right);
    roi_proro->mutable_model_roi()->set_bottom(roi_data.model_roi.bottom);
  }

  image_proto->set_perception_mode(image_data.perception_mode);
  image_proto->set_postprocessor_status(image_data.postprocessor_status);
  image_proto->set_frame_id(image_data.frame_id);
  image_proto->set_step(image_data.step);
  image_proto->set_time_stamp_us(image_data.time_stamp_us);
  image_proto->set_hw_time_stamp(image_data.hw_time_stamp);
  image_proto->set_mcu_time_diff(image_data.mcu_time_diff);
  image_proto->set_mcu_board_timediff(image_data.mcu_board_timediff);

  workflow::proto::ParsingSetting* parse_set_proto =
      proto_msg->mutable_parsing_setting();
  auto& parse_set_data{parsing_data.parsing_setting};

  parse_set_proto->mutable_header()->set_cam_idx(parse_set_data.header.cam_idx);
  parse_set_proto->mutable_header()->set_frame_id(
      parse_set_data.header.frame_id);
  parse_set_proto->mutable_header()->set_time_stamp(
      parse_set_data.header.time_stamp);
  parse_set_proto->set_parsing_type(parse_set_data.parsing_type);
  parse_set_proto->set_parsing_channels(parse_set_data.parsing_channels);

  for (std::uint8_t cl_idx = 0; cl_idx < parse_set_data.num_channel_labels;
       cl_idx++) {
    parse_set_proto->set_parsing_channels(
        parse_set_data.channel_labels.at(cl_idx));
  }

  for (std::uint8_t cc_idx = 0; cc_idx < parse_set_data.num_channel_colors;
       cc_idx++) {
    auto& cc_data{parse_set_data.channel_colors.at(cc_idx)};
    parse_set_proto->mutable_channel_colors(cc_idx)->set_b(cc_data.b);
    parse_set_proto->mutable_channel_colors(cc_idx)->set_r(cc_data.r);
    parse_set_proto->mutable_channel_colors(cc_idx)->set_g(cc_data.g);
  }

  for (std::uint8_t fc_idx = 0; fc_idx < parse_set_data.num_freespace_channels;
       fc_idx++) {
    parse_set_proto->set_freespace_channels(
        fc_idx, parse_set_data.freespace_channels.at(fc_idx));
  }

  for (std::uint8_t bc_idx = 0; bc_idx < parse_set_data.num_boundspace_channels;
       bc_idx++) {
    parse_set_proto->set_boundspace_channels(
        bc_idx, parse_set_data.boundspace_channels.at(bc_idx));
  }

  parse_set_proto->mutable_parsing_roi()->set_bottom(
      parse_set_data.parsing_roi.bottom);
  parse_set_proto->mutable_parsing_roi()->set_left(
      parse_set_data.parsing_roi.left);
  parse_set_proto->mutable_parsing_roi()->set_right(
      parse_set_data.parsing_roi.right);
  parse_set_proto->mutable_parsing_roi()->set_top(
      parse_set_data.parsing_roi.top);

  parse_set_proto->set_parsing_scale(parse_set_data.parsing_scale);

  for (std::uint8_t mi_idx = 0; mi_idx < parse_set_data.num_model_info;
       mi_idx++) {
    workflow::proto::RuntimeModelInfo* mi_proto =
        parse_set_proto->mutable_model_info(mi_idx);
    auto& mi_data{parse_set_data.model_info.at(mi_idx)};
    mi_proto->set_model_id(mi_data.model_id);
    mi_proto->set_model_name(mi_data.model_name.data(), mi_data.num_model_name);
  }

  for (std::uint8_t c_idx = 0; c_idx < parse_set_data.num_categories; c_idx++) {
    workflow::proto::perception_base::Category* c_proto =
        parse_set_proto->mutable_categories(c_idx);
    auto& c_data{parse_set_data.categories.at(c_idx)};
    c_proto->set_property_type(c_data.property_type);
    c_proto->set_property(c_data.property);
    c_proto->set_property_conf(c_data.property_conf);
    c_proto->set_property_name(c_data.property_name);
  }

  workflow::proto::DepthSetting* depth_set_proto =
      proto_msg->mutable_depth_setting();
  auto& depth_set_data{parsing_data.depth_setting};
  depth_set_proto->set_scale_number(depth_set_data.scale_number);
  depth_set_proto->set_shift_scale(depth_set_data.shift_scale);
  depth_set_proto->set_mapping_type(depth_set_data.mapping_type);
  depth_set_proto->set_confidence_scale(depth_set_data.confidence_scale);
  depth_set_proto->set_max_depth(depth_set_data.max_depth);
  depth_set_proto->set_min_depth(depth_set_data.min_depth);
  depth_set_proto->set_max_confidence(depth_set_data.max_confidence);

  workflow::proto::CameraParam* camera_proto =
      depth_set_proto->mutable_camera_param();
  auto& camera_data{depth_set_data.camera_param};
  camera_proto->set_focal_u(camera_data.focal_u);
  camera_proto->set_focal_v(camera_data.focal_v);
  camera_proto->set_center_u(camera_data.center_u);
  camera_proto->set_center_v(camera_data.center_v);
  camera_proto->set_camera_x(camera_data.camera_x);
  camera_proto->set_camera_y(camera_data.camera_y);
  camera_proto->set_camera_z(camera_data.camera_z);
  camera_proto->set_pitch(camera_data.pitch);
  camera_proto->set_yaw(camera_data.yaw);
  camera_proto->set_roll(camera_data.roll);
  camera_proto->set_type(camera_data.type);
  camera_proto->set_fov(camera_data.fov);

  camera_proto->set_version(camera_data.version);
  camera_proto->set_cali_status(
      static_cast<workflow::proto::ParsingCalibrationStatus>(
          camera_data.cali_status));
  camera_proto->mutable_valid_height()->set_left_y(
      camera_data.valid_height.left_y);
  camera_proto->mutable_valid_height()->set_right_y(
      camera_data.valid_height.right_y);

  for (std::uint8_t d_idx = 0; d_idx < camera_data.distort.num_param; d_idx++) {
    camera_proto->mutable_distort()->add_param(
        camera_data.distort.param.at(d_idx));
  }

  workflow::proto::VCSParam* vcs_proto = camera_proto->mutable_vcs();
  auto& vcs_data{camera_data.vcs};
  for (std::uint8_t r_idx = 0; r_idx < vcs_data.num_rotation; r_idx++) {
    vcs_proto->add_rotation(vcs_data.rotation.at(r_idx));
  }
  for (std::uint8_t t_idx = 0; t_idx < vcs_data.num_translation; t_idx++) {
    vcs_proto->add_translation(vcs_data.translation.at(t_idx));
  }

  workflow::proto::perception_base::CameraMatrix* mat_proto =
      camera_proto->mutable_mat();
  auto& mat_data{camera_data.mat};
  for (size_t gi_idx = 0; gi_idx < mat_data.mat_gnd2img.size(); gi_idx++) {
    mat_proto->add_mat_gnd2img(mat_data.mat_gnd2img.at(gi_idx));
  }
  for (size_t ig_idx = 0; ig_idx < mat_data.mat_img2gnd.size(); ig_idx++) {
    mat_proto->add_mat_img2gnd(mat_data.mat_img2gnd.at(ig_idx));
  }
  for (size_t vgi_idx = 0; vgi_idx < mat_data.mat_vcsgnd2img.size();
       vgi_idx++) {
    mat_proto->add_mat_vcsgnd2img(mat_data.mat_vcsgnd2img.at(vgi_idx));
  }
  for (size_t ivg_idx = 0; ivg_idx < mat_data.mat_img2vcsgnd.size();
       ivg_idx++) {
    mat_proto->add_mat_img2vcsgnd(mat_data.mat_img2vcsgnd.at(ivg_idx));
  }
  for (size_t li_idx = 0; li_idx < mat_data.mat_local2img.size(); li_idx++) {
    mat_proto->add_mat_local2img(mat_data.mat_local2img.at(li_idx));
  }
  for (size_t il_idx = 0; il_idx < mat_data.mat_img2local.size(); il_idx++) {
    mat_proto->add_mat_img2local(mat_data.mat_img2local.at(il_idx));
  }
  for (size_t vi_idx = 0; vi_idx < mat_data.mat_vcs2img.size(); vi_idx++) {
    mat_proto->add_mat_vcs2img(mat_data.mat_vcs2img.at(vi_idx));
  }

  camera_proto->set_time_stamp(camera_data.time_stamp);
  camera_proto->set_cam_id(camera_data.cam_id);
  camera_proto->set_frame_id(camera_data.frame_id);
  camera_proto->set_image_height(camera_data.image_height);
  camera_proto->set_image_width(camera_data.image_width);
  camera_proto->set_base_calib_done(camera_data.base_calib_done);

  camera_proto->set_calib_src(
      static_cast<workflow::proto::CalibrationType>(camera_data.calib_src));
  camera_proto->set_calib_done_ts(camera_data.calib_done_ts);

  return 0;
}

int ParsingSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                   message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& parse_data{data_msg->MutableParsing()};

  if (proto_msg->has_free_space()) {
    auto& free_space_data{parse_data.free_space};
    auto& free_space_proto{proto_msg->free_space()};

    auto& str_value{free_space_proto.data()};
    if (str_value.length() > 200704U) {
      throw OutOfResourceException("parsing free_space_data->data \
            info size too long");
    }

    memcpy(free_space_data.data, str_value.c_str(),
           str_value.length());
    free_space_data.width = free_space_proto.width();
    free_space_data.height = free_space_proto.height();
    free_space_data.channel = free_space_proto.channel();
  }  // end free_space

  if (proto_msg->has_feature_map()) {
    auto& feature_map_data{parse_data.feature_map};
    auto& feature_map_proto{proto_msg->feature_map()};
    auto& str_value{feature_map_proto.data()};
    if (str_value.length() > 200704U) {
      throw OutOfResourceException("parsing feature_map_data->data \
            info size too long");
    }
    memcpy(feature_map_data.data, str_value.c_str(), str_value.length());
    feature_map_data.width = feature_map_proto.width();
    feature_map_data.height = feature_map_proto.height();
    feature_map_data.channel = feature_map_proto.channel();
  }  // end feature_map

  parse_data.scale = proto_msg->scale();

  if (proto_msg->has_roi_output()) {
    auto& roi_data{parse_data.roi_output};
    auto& roi_proto{proto_msg->roi_output()};
    roi_data.bottom = roi_proto.bottom();
    roi_data.left = roi_proto.left();
    roi_data.right = roi_proto.right();
    roi_data.top = roi_proto.top();
  }  // end roi_output

  if (proto_msg->has_image()) {
    auto& image_data{parse_data.image};
    auto& image_proto{proto_msg->image()};
    image_data.width = image_proto.width();
    image_data.height = image_proto.height();
    image_data.channel = image_proto.channel();
    image_data.send_mod = image_proto.send_mod();
    image_data.time_stamp = image_proto.time_stamp();

    image_data.format = image_proto.format();
    image_data.index = image_proto.index();
    image_data.count = image_proto.count();
    image_data.depth = image_proto.depth();
    image_data.align = image_proto.align();

    image_data.image_counter = image_proto.image_counter();
    image_data.perception_status = image_proto.perception_status();
    image_data.bypass_time_diff = image_proto.bypass_time_diff();
    image_data.compression_format = image_proto.compression_format();
    image_data.status = image_proto.status();

    image_data.num_roi_map = image_proto.roi_map_size();
    for (std::uint8_t rm_idx = 0; rm_idx < image_data.num_roi_map; rm_idx++) {
      auto& roi_data{image_data.roi_map.at(rm_idx)};
      roi_data.model_roi.bottom =
          image_proto.roi_map(rm_idx).model_roi().bottom();
      roi_data.model_roi.left = image_proto.roi_map(rm_idx).model_roi().left();
      roi_data.model_roi.right =
          image_proto.roi_map(rm_idx).model_roi().right();
      roi_data.model_roi.top = image_proto.roi_map(rm_idx).model_roi().top();
      roi_data.model_id = image_proto.roi_map(rm_idx).model_id();
    }  // end roi_map

    image_data.perception_mode = image_proto.perception_mode();
    image_data.postprocessor_status = image_proto.postprocessor_status();
    image_data.frame_id = image_proto.frame_id();
    image_data.step = image_proto.step();
    image_data.time_stamp_us = image_proto.time_stamp_us();
    image_data.hw_time_stamp = image_proto.hw_time_stamp();
    image_data.mcu_time_diff = image_proto.mcu_time_diff();
    image_data.mcu_board_timediff = image_proto.mcu_board_timediff();
  }  // end image

  if (proto_msg->has_parsing_setting()) {
    auto& parse_set_data{parse_data.parsing_setting};
    auto& parse_set_proto{proto_msg->parsing_setting()};
    parse_set_data.header.cam_idx = parse_set_proto.header().cam_idx();
    parse_set_data.header.frame_id = parse_set_proto.header().frame_id();
    parse_set_data.header.time_stamp = parse_set_proto.header().time_stamp();
    parse_set_data.parsing_type = parse_set_proto.parsing_type();
    parse_set_data.parsing_channels = parse_set_proto.parsing_channels();

    parse_set_data.num_channel_labels = parse_set_proto.channel_labels_size();
    for (std::uint8_t cl_idx = 0; cl_idx < parse_set_data.num_channel_labels;
         cl_idx++) {
      parse_set_data.channel_labels.at(cl_idx) =
          parse_set_proto.channel_labels(cl_idx);
    }

    parse_set_data.num_channel_colors = parse_set_proto.channel_colors_size();
    for (std::uint8_t cc_idx = 0; cc_idx < parse_set_data.num_channel_colors;
         cc_idx++) {
      parse_set_data.channel_colors.at(cc_idx).r =
          parse_set_proto.channel_colors(cc_idx).r();
      parse_set_data.channel_colors.at(cc_idx).g =
          parse_set_proto.channel_colors(cc_idx).g();
      parse_set_data.channel_colors.at(cc_idx).b =
          parse_set_proto.channel_colors(cc_idx).b();
    }

    parse_set_data.num_freespace_channels =
        parse_set_proto.freespace_channels_size();
    for (std::uint8_t fc_idx = 0;
         fc_idx < parse_set_data.num_freespace_channels; fc_idx++) {
      parse_set_data.freespace_channels.at(fc_idx) =
          parse_set_proto.freespace_channels(fc_idx);
    }

    parse_set_data.num_boundspace_channels =
        parse_set_proto.boundspace_channels_size();
    for (std::uint8_t bc_idx = 0;
         bc_idx < parse_set_data.num_boundspace_channels; bc_idx++) {
      parse_set_data.boundspace_channels.at(bc_idx) =
          parse_set_proto.boundspace_channels(bc_idx);
    }

    if (parse_set_proto.has_parsing_roi()) {
      auto& roi_data{parse_set_data.parsing_roi};
      auto& roi_proto{parse_set_proto.parsing_roi()};
      roi_data.bottom = roi_proto.bottom();
      roi_data.left = roi_proto.left();
      roi_data.right = roi_proto.right();
      roi_data.top = roi_proto.top();
    }  // end parsing_roi

    parse_set_data.parsing_scale = parse_set_proto.parsing_scale();

    parse_set_data.num_model_info = parse_set_proto.model_info_size();
    for (std::uint8_t mi_idx = 0; mi_idx < parse_set_data.num_model_info;
         mi_idx++) {
      auto& model_info_data{parse_set_data.model_info.at(mi_idx)};
      auto& model_info_proto{parse_set_proto.model_info(mi_idx)};
      model_info_data.model_id = model_info_proto.model_id();
      std::string model_mane{model_info_proto.model_name()};
      memcpy(model_info_data.model_name.data(), model_mane.c_str(),
             model_mane.length());
    }  // end model_info

    parse_set_data.num_categories = parse_set_proto.categories_size();
    for (std::uint8_t c_idx = 0; c_idx < parse_set_data.num_categories;
         c_idx++) {
      auto& cate_data{parse_set_data.categories.at(c_idx)};
      auto& cate_proto{parse_set_proto.categories(c_idx)};
      cate_data.property_type = cate_proto.property_type();
      cate_data.property = cate_proto.property();
      cate_data.property_conf = cate_proto.property_conf();
      std::string property_name{cate_proto.property_name()};
      memcpy(cate_data.property_name, property_name.c_str(),
             property_name.length());
    }
  }  // end parsing_setting

  if (proto_msg->has_depth_setting()) {
    auto& depth_set_data{parse_data.depth_setting};
    auto& depth_set_proto{proto_msg->depth_setting()};
    depth_set_data.scale_number = depth_set_proto.scale_number();
    depth_set_data.shift_scale = depth_set_proto.shift_scale();
    depth_set_data.mapping_type = depth_set_proto.mapping_type();
    depth_set_data.mapping_shift = depth_set_proto.mapping_shift();
    depth_set_data.confidence_scale = depth_set_proto.confidence_scale();
    depth_set_data.max_depth = depth_set_proto.max_depth();
    depth_set_data.min_depth = depth_set_proto.min_depth();
    depth_set_data.max_confidence = depth_set_proto.max_confidence();
    depth_set_data.min_confidence = depth_set_proto.min_confidence();

    if (depth_set_proto.has_camera_param()) {
      auto& camera_data{depth_set_data.camera_param};
      auto& camera_proto{depth_set_proto.camera_param()};
      camera_data.focal_u = camera_proto.focal_u();
      camera_data.focal_v = camera_proto.focal_v();
      camera_data.center_u = camera_proto.center_u();
      camera_data.center_v = camera_proto.center_v();
      camera_data.camera_x = camera_proto.camera_x();
      camera_data.camera_y = camera_proto.camera_y();
      camera_data.camera_z = camera_proto.camera_z();
      camera_data.pitch = camera_proto.pitch();
      camera_data.yaw = camera_proto.yaw();
      camera_data.roll = camera_proto.roll();
      camera_data.type = camera_proto.type();
      camera_data.fov = camera_proto.fov();
      camera_data.version = camera_proto.version();
      camera_data.cali_status =
          static_cast<ParsingData::ParsingCalibrationStatus>(
              camera_proto.cali_status());
      camera_data.valid_height.left_y = camera_proto.valid_height().left_y();
      camera_data.valid_height.right_y = camera_proto.valid_height().right_y();

      if (camera_proto.has_distort()) {
        auto& distort{camera_data.distort};
        distort.num_param = camera_proto.distort().param_size();
        for (std::uint8_t d_idx = 0; d_idx < distort.num_param; d_idx++) {
          distort.param.at(d_idx) = camera_proto.distort().param(d_idx);
        }
      }  // end distort

      if (camera_proto.has_vcs()) {
        auto& vcs_data{camera_data.vcs};
        auto& vcs_proto{camera_proto.vcs()};

        vcs_data.num_rotation = vcs_proto.rotation_size();
        for (std::uint8_t r_idx = 0; r_idx < vcs_data.num_rotation; r_idx++) {
          vcs_data.rotation.at(r_idx) = vcs_proto.rotation(r_idx);
        }

        vcs_data.num_translation = vcs_proto.translation_size();
        for (std::uint8_t t_idx = 0; t_idx < vcs_data.num_translation;
             t_idx++) {
          vcs_data.translation.at(t_idx) = vcs_proto.translation(t_idx);
        }
      }  // end vcs

      if (camera_proto.has_mat()) {
        auto& mat_data{camera_data.mat};
        auto& mat_proto{camera_proto.mat()};

        for (std::int32_t gi_idx = 0; gi_idx < mat_proto.mat_gnd2img_size();
             gi_idx++) {
          mat_data.mat_gnd2img.at(gi_idx) = mat_proto.mat_gnd2img(gi_idx);
        }

        for (std::int32_t ig_idx = 0; ig_idx < mat_proto.mat_img2gnd_size();
             ig_idx++) {
          mat_data.mat_img2gnd.at(ig_idx) = mat_proto.mat_img2gnd(ig_idx);
        }

        for (std::int32_t vgi_idx = 0;
             vgi_idx < mat_proto.mat_vcsgnd2img_size(); vgi_idx++) {
          mat_data.mat_vcsgnd2img.at(vgi_idx) =
              mat_proto.mat_vcsgnd2img(vgi_idx);
        }

        for (std::int32_t ivg_idx = 0;
             ivg_idx < mat_proto.mat_img2vcsgnd_size(); ivg_idx++) {
          mat_data.mat_img2vcsgnd.at(ivg_idx) =
              mat_proto.mat_img2vcsgnd(ivg_idx);
        }

        for (std::int32_t li_idx = 0; li_idx < mat_proto.mat_local2img_size();
             li_idx++) {
          mat_data.mat_local2img.at(li_idx) = mat_proto.mat_local2img(li_idx);
        }

        for (std::int32_t il_idx = 0; il_idx < mat_proto.mat_img2local_size();
             il_idx++) {
          mat_data.mat_img2local.at(il_idx) = mat_proto.mat_img2local(il_idx);
        }

        for (std::int32_t vi_idx = 0; vi_idx < mat_proto.mat_vcs2img_size();
             vi_idx++) {
          mat_data.mat_vcs2img.at(vi_idx) = mat_proto.mat_vcs2img(vi_idx);
        }
      }  // end mat

      camera_data.time_stamp = camera_proto.time_stamp();
      camera_data.cam_id = camera_proto.cam_id();
      camera_data.frame_id = camera_proto.frame_id();
      camera_data.image_height = camera_proto.image_height();
      camera_data.image_width = camera_proto.image_width();
      camera_data.base_calib_done = camera_proto.base_calib_done();
      camera_data.calib_src =
          static_cast<ParsingData::CalibrationType>(camera_proto.calib_src());
      camera_data.calib_done_ts = camera_proto.calib_done_ts();
    }  // end camera_param
  }    // end depth_setting
  return 0;
}

}  // namespace autodrive