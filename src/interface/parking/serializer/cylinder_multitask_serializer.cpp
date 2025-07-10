//

#include "interface/parking/serializer/cylinder_multitask_serializer.h"

#include <utils/exception.h>

#include "interface/parking/cylinder_multitask_data.h"
#include "protocol/cylinder_multitask.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/dynamic_obstacle_serializer.h"

namespace autodrive {

int CylinderMultiTaskSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& cylinder_multitask_data{data_msg->GetCylinderMultiTask()};
  if (static_cast<std::uint32_t>(cylinder_multitask_data.num_camera_flag_data) >
      CylinderMultiTaskData::kMaxCameraNumber) {
    throw OutOfResourceException("Cylinder multitask data info size too long");
  }

  int num_obstacle_raw_data = 0;
  int num_real3d_data = 0;
  int num_parsing_data = 0;
  for (std::uint8_t idx = 0;
       idx < cylinder_multitask_data.num_camera_flag_data; ++idx) {
    auto const& valid_flag{cylinder_multitask_data.camera_flag_data.at(idx)};
    workflow::proto::CameraValidFlag* valid_proto = 
      proto_msg->add_camera_flag_data();
      valid_proto->set_sensor_id(
        static_cast<workflow::proto::perception_base::ImageId>(valid_flag.sensor_id));
      valid_proto->set_obstacle_raw_data_valid(valid_flag.obstacle_raw_data_valid);
      num_obstacle_raw_data += valid_flag.obstacle_raw_data_valid ? 1 : 0;
      valid_proto->set_real3d_data_valid(valid_flag.real3d_data_valid);
      num_real3d_data += valid_flag.real3d_data_valid ? 1 : 0;
      valid_proto->set_parsing_data_valid(valid_flag.parsing_data_valid);
      num_parsing_data += valid_flag.parsing_data_valid ? 1 : 0;
  }

  for (std::uint8_t idx = 0; idx < num_obstacle_raw_data; ++idx) {
    {
      auto& raw_data{cylinder_multitask_data.obstacle_raw_data.at(idx)};
      workflow::proto::ObstacleRawsData* raw_proto =
          proto_msg->add_obstacle_raw_data();
      raw_proto->mutable_header()->set_cam_idx(raw_data.header.cam_idx);
      raw_proto->mutable_header()->set_frame_id(raw_data.header.frame_id);
      raw_proto->mutable_header()->set_time_stamp(raw_data.header.time_stamp);

      {
        auto& category_data{raw_data.category};
        workflow::proto::perception_base::Category* category_proto =
            raw_proto->mutable_category();
        category_proto->set_property_type(category_data.property_type);
        category_proto->set_property(category_data.property);
        category_proto->set_property_conf(category_data.property_conf);
        category_proto->set_property_name(category_data.property_name);
      }  // end category

      if (static_cast<std::uint32_t>(raw_data.num_obstacle_raws) > kMaxObstacleRaws) {
        throw OutOfResourceException("Cylinder multitask data \
              -> obstacle_raws info size too long");
      }

      for (std::uint8_t raw_idx = 0; raw_idx < raw_data.num_obstacle_raws;
           raw_idx++) {
        auto& obst_data{raw_data.obstacle_raws.at(raw_idx)};
        workflow::proto::ObstacleRaw* obst_proto =
            raw_proto->add_obstacle_raws();

        obst_proto->mutable_rect()->set_bottom(obst_data.rect.bottom);
        obst_proto->mutable_rect()->set_left(obst_data.rect.left);
        obst_proto->mutable_rect()->set_right(obst_data.rect.right);
        obst_proto->mutable_rect()->set_top(obst_data.rect.top);

        obst_proto->set_conf(obst_data.conf);
        obst_proto->set_model(obst_data.model);
        obst_proto->set_source(obst_data.source);

        if (static_cast<std::uint32_t>(obst_data.num_category) > kMaxObstacleRawCategorySize) {
          throw OutOfResourceException("Cylinder multitask data \
                -> obstacle_raws -> category info size too long");
        }

        for (std::uint8_t obst_idx = 0; obst_idx < obst_data.num_category;
             obst_idx++) {
          auto& obst_category_data{obst_data.category.at(obst_idx)};
          workflow::proto::perception_base::Category* obst_category_proto =
              obst_proto->add_category();
          obst_category_proto->set_property_type(
              obst_category_data.property_type);
          obst_category_proto->set_property(obst_category_data.property);
          obst_category_proto->set_property_conf(
              obst_category_data.property_conf);
          obst_category_proto->set_property_name(
              obst_category_data.property_name);
        }  // end for - category

        for (std::uint8_t p_idx = 0; p_idx < obst_data.num_property_bbox;
             p_idx++) {
          auto& property_data{obst_data.property_bbox.at(p_idx)};
          workflow::proto::PropertyPerceptionBBox* property_proto =
              obst_proto->add_property_bbox();

          property_proto->mutable_rect()->set_bottom(property_data.rect.bottom);
          property_proto->mutable_rect()->set_left(property_data.rect.left);
          property_proto->mutable_rect()->set_right(property_data.rect.right);
          property_proto->mutable_rect()->set_top(property_data.rect.top);

          property_proto->set_conf(property_data.conf);
          property_proto->set_type(property_data.type);

          for (std::uint8_t pc_idx = 0; pc_idx < property_data.num_category;
               pc_idx++) {
            auto& p_category_data{property_data.category.at(pc_idx)};
            workflow::proto::perception_base::Category* p_category_proto =
                property_proto->add_category();

            p_category_proto->set_property_type(p_category_data.property_type);
            p_category_proto->set_property(p_category_data.property);
            p_category_proto->set_property_conf(p_category_data.property_conf);
            p_category_proto->set_property_name(p_category_data.property_name);
          }  // end for - category
        }    // end for - property_bbox

        for (size_t kp_idx = 0; kp_idx < obst_data.key_points.size();
             kp_idx++) {
          auto& key_point_data{obst_data.key_points.at(kp_idx)};
          workflow::proto::ObstacleRawKeyPoint* key_point_proto =
              obst_proto->add_key_points();

          key_point_proto->mutable_pt()->set_x(key_point_data.pt.x);
          key_point_proto->mutable_pt()->set_y(key_point_data.pt.y);
          key_point_proto->mutable_pt()->set_z(key_point_data.pt.z);

          key_point_proto->set_conf(key_point_data.conf);
          key_point_proto->set_type(key_point_data.type);
          key_point_proto->set_boundary_conf_x(key_point_data.boundary_conf_x);
          key_point_proto->set_boundary_conf_y(key_point_data.boundary_conf_y);
        }  // end for - key_points

        obst_proto->set_depth(obst_data.depth);
        obst_proto->set_expand(obst_data.expand);

        obst_proto->mutable_input_roi()->set_bottom(obst_data.input_roi.bottom);
        obst_proto->mutable_input_roi()->set_left(obst_data.input_roi.left);
        obst_proto->mutable_input_roi()->set_right(obst_data.input_roi.right);
        obst_proto->mutable_input_roi()->set_top(obst_data.input_roi.top);
      }    // end for obstacle_raws

      if (static_cast<std::uint32_t>(raw_data.num_key_points_raws) > kStageOneKeyPointsSize) {
        throw OutOfResourceException("Cylinder multitask data \
              -> num_key_points_raws info size too long");
      }
      for (std::uint8_t kp_raw_idx = 0; kp_raw_idx< raw_data.num_key_points_raws;
            kp_raw_idx++) {
        auto& kp_raw_data{raw_data.key_points_raws.at(kp_raw_idx)};
        workflow::proto::KeyPointsRawData* kp_raw_proto =
            raw_proto->add_key_points_raws();
        kp_raw_proto->set_type(static_cast<workflow::proto::KeyPointType>(kp_raw_data.type));

        if (static_cast<std::uint32_t>(kp_raw_data.num_key_points) > kMaxKeyPoints) {
          throw OutOfResourceException("Cylinder multitask data \
                -> key_points info size too long");
        }
        for (size_t kp_idx = 0; kp_idx < kp_raw_data.num_key_points;
          kp_idx++) {
          auto& key_point_data{kp_raw_data.key_points.at(kp_idx)};
          workflow::proto::ObstacleRawKeyPoint* key_point_proto =
              kp_raw_proto->add_key_points();

          key_point_proto->mutable_pt()->set_x(key_point_data.pt.x);
          key_point_proto->mutable_pt()->set_y(key_point_data.pt.y);
          key_point_proto->mutable_pt()->set_z(key_point_data.pt.z);

          key_point_proto->set_conf(key_point_data.conf);
          key_point_proto->set_type(key_point_data.type);
          key_point_proto->set_boundary_conf_x(key_point_data.boundary_conf_x);
          key_point_proto->set_boundary_conf_y(key_point_data.boundary_conf_y);
        }  // end for - key_points
      }
    }      // end obstacle_raw_data
  }

  for (std::uint8_t idx = 0; idx < num_real3d_data; ++idx) {
    {
      auto& real3d_data{cylinder_multitask_data.real3d_data.at(idx)};
      workflow::proto::Real3DData* real3d_proto = proto_msg->add_real3d_data();

      real3d_proto->mutable_header()->set_cam_idx(real3d_data.header.cam_idx);
      real3d_proto->mutable_header()->set_frame_id(real3d_data.header.frame_id);
      real3d_proto->mutable_header()->set_time_stamp(
          real3d_data.header.time_stamp);

      for (size_t rr_idx = 0; rr_idx < real3d_data.real3d_results.size();
           rr_idx++) {
        auto& result_data{real3d_data.real3d_results.at(rr_idx)};
        workflow::proto::Real3DResults* result_proto =
            real3d_proto->add_real3d_results();

        result_proto->mutable_header()->set_cam_idx(result_data.header.cam_idx);
        result_proto->mutable_header()->set_frame_id(
            result_data.header.frame_id);
        result_proto->mutable_header()->set_time_stamp(
            result_data.header.time_stamp);

        result_proto->set_type(
            static_cast<workflow::proto::Real3DType>(result_data.type));

        result_proto->set_coordinate_type(result_data.coordinate_type.data(),
                                          result_data.num_coordinate_type);

        for (std::uint8_t rd_idx = 0; rd_idx < result_data.num_real3d_result;
             rd_idx++) {
          auto& rd_data{result_data.real3d_result.at(rd_idx)};
          workflow::proto::Real3DResult* rd_proto =
              result_proto->add_real3d_result();

          {
            auto& bev_obst_data{rd_data.bev_obstacle};
            workflow::proto::DynamicObstacle* bev_obst_proto =
                rd_proto->mutable_bev_obstacle();

            DynamicObstacleSerialize(bev_obst_data, bev_obst_proto);
          }    // end bev_obstacle
          {
            auto& box_data{rd_data.img_3dbox};
            workflow::proto::perception_base::Box3D* box_proto =
                rd_proto->mutable_img_3dbox();

            box_proto->mutable_lower_lt()->set_x(box_data.lower_lt.x);
            box_proto->mutable_lower_lt()->set_y(box_data.lower_lt.y);
            box_proto->mutable_lower_lt()->set_z(box_data.lower_lt.z);

            box_proto->mutable_lower_lb()->set_x(box_data.lower_lb.x);
            box_proto->mutable_lower_lb()->set_y(box_data.lower_lb.y);
            box_proto->mutable_lower_lb()->set_z(box_data.lower_lb.z);

            box_proto->mutable_lower_rb()->set_x(box_data.lower_rb.x);
            box_proto->mutable_lower_rb()->set_y(box_data.lower_rb.y);
            box_proto->mutable_lower_rb()->set_z(box_data.lower_rb.z);

            box_proto->mutable_lower_rt()->set_x(box_data.lower_rt.x);
            box_proto->mutable_lower_rt()->set_y(box_data.lower_rt.y);
            box_proto->mutable_lower_rt()->set_z(box_data.lower_rt.z);

            box_proto->mutable_upper_lt()->set_x(box_data.upper_lt.x);
            box_proto->mutable_upper_lt()->set_y(box_data.upper_lt.y);
            box_proto->mutable_upper_lt()->set_z(box_data.upper_lt.z);

            box_proto->mutable_upper_lb()->set_x(box_data.upper_lb.x);
            box_proto->mutable_upper_lb()->set_y(box_data.upper_lb.y);
            box_proto->mutable_upper_lb()->set_z(box_data.upper_lb.z);

            box_proto->mutable_upper_rb()->set_x(box_data.upper_rb.x);
            box_proto->mutable_upper_rb()->set_y(box_data.upper_rb.y);
            box_proto->mutable_upper_rb()->set_z(box_data.upper_rb.z);

            box_proto->mutable_upper_rt()->set_x(box_data.upper_rt.x);
            box_proto->mutable_upper_rt()->set_y(box_data.upper_rt.y);
            box_proto->mutable_upper_rt()->set_z(box_data.upper_rt.z);

            box_proto->set_conf(box_data.conf);
          }  // end img_3dbox

          {
            auto& img_rect_data{rd_data.img_rect};
            workflow::proto::perception_base::Rect* img_rect_proto =
                rd_proto->mutable_img_rect();
            img_rect_proto->set_bottom(img_rect_data.bottom);
            img_rect_proto->set_left(img_rect_data.left);
            img_rect_proto->set_right(img_rect_data.right);
            img_rect_proto->set_top(img_rect_data.top);
          }  // end img_rect
        }    // end for real3d_result
      }      // end for - real3d_results
    }        // end for - real3d_data
  }
  return 0;
}

int CylinderMultiTaskSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& cylinder_multitask_data{data_msg->MutableCylinderMultiTask()};

  cylinder_multitask_data.num_camera_flag_data =
      proto_msg->camera_flag_data_size();

  if (static_cast<std::uint32_t>(
          cylinder_multitask_data.num_camera_flag_data) >
          CylinderMultiTaskData::kMaxCameraNumber) {
    throw OutOfResourceException("Cylinder multitask data info size too long");
  }

  for (std::uint8_t idx = 0;
       ((idx < cylinder_multitask_data.num_camera_flag_data) &&
        (idx < CylinderMultiTaskData::kMaxCameraNumber));
       idx++) {
    auto& valid_data{cylinder_multitask_data.camera_flag_data.at(idx)};
    auto& valid_proto{proto_msg->camera_flag_data(idx)};

    valid_data.sensor_id = static_cast<autodrive::ImageId>(valid_proto.sensor_id());
    valid_data.obstacle_raw_data_valid = valid_proto.obstacle_raw_data_valid();
    valid_data.real3d_data_valid = valid_proto.real3d_data_valid();
    valid_data.parsing_data_valid = valid_proto.parsing_data_valid();
  }

  std::uint8_t num_obstacle_raw_data =
              static_cast<std::uint8_t>(proto_msg->obstacle_raw_data_size());
  for (std::uint8_t idx = 0;
       ((idx < num_obstacle_raw_data) &&
        (idx < CylinderMultiTaskData::kMaxCameraNumber));
       idx++) {
    {
      auto& obst_raw_data{cylinder_multitask_data.obstacle_raw_data.at(idx)};
      auto& obst_raw_proto{proto_msg->obstacle_raw_data(idx)};

      if (obst_raw_proto.has_header()) {
        obst_raw_data.header.cam_idx = obst_raw_proto.header().cam_idx();
        obst_raw_data.header.frame_id = obst_raw_proto.header().frame_id();
        obst_raw_data.header.time_stamp = obst_raw_proto.header().time_stamp();
      }  // end header

      if (obst_raw_proto.has_category()) {
        obst_raw_data.category.property_type =
            obst_raw_proto.category().property_type();
        obst_raw_data.category.property = obst_raw_proto.category().property();
        obst_raw_data.category.property_conf =
            obst_raw_proto.category().property_conf();

        auto& str_value{obst_raw_proto.category().property_name()};
        memcpy(obst_raw_data.category.property_name, str_value.c_str(),
               str_value.length());
      }  // end category

      obst_raw_data.conf_scale = obst_raw_proto.conf_scale();
      obst_raw_data.num_obstacle_raws = obst_raw_proto.obstacle_raws_size();
      for (std::uint8_t obs_idx = 0; obs_idx < obst_raw_data.num_obstacle_raws;
           obs_idx++) {
        auto& obst_data{obst_raw_data.obstacle_raws.at(obs_idx)};
        auto& obst_proto{obst_raw_proto.obstacle_raws(obs_idx)};

        if (obst_proto.has_rect()) {
          obst_data.rect.bottom = obst_proto.rect().bottom();
          obst_data.rect.right = obst_proto.rect().right();
          obst_data.rect.left = obst_proto.rect().left();
          obst_data.rect.top = obst_proto.rect().top();
        }  // end rect

        obst_data.conf = obst_proto.conf();
        obst_data.model = obst_proto.model();

        obst_data.num_category =
            static_cast<std::uint8_t>(obst_proto.category_size());
        for (std::uint8_t ct_idx = 0; ct_idx < obst_data.num_category;
             ++ct_idx) {
          auto& category_data{obst_data.category.at(ct_idx)};
          auto& category_proto{obst_proto.category(ct_idx)};

          category_data.property_type = category_proto.property_type();
          category_data.property = category_proto.property();
          category_data.property_conf = category_proto.property_conf();

          auto& str_value{category_proto.property_name()};
          memcpy(category_data.property_name, str_value.c_str(),
                 str_value.length());
        }  // end for - category

        obst_data.num_property_bbox =
            static_cast<std::uint8_t>(obst_proto.property_bbox_size());
        for (std::uint8_t pb_idx = 0; pb_idx < obst_data.num_property_bbox;
             pb_idx++) {
          auto& pb_data{obst_data.property_bbox.at(pb_idx)};
          auto& pb_proto{obst_proto.property_bbox(pb_idx)};

          if (pb_proto.has_rect()) {
            pb_data.rect.bottom = pb_proto.rect().bottom();
            pb_data.rect.left = pb_proto.rect().left();
            pb_data.rect.right = pb_proto.rect().right();
            pb_data.rect.top = pb_proto.rect().top();
          }

          pb_data.conf = pb_proto.conf();
          pb_data.type = pb_proto.type();

          pb_data.num_category =
              static_cast<std::uint8_t>(pb_proto.category_size());
          for (std::uint8_t pc_idx = 0; pc_idx < pb_data.num_category;
               pc_idx++) {
            auto& category_data{pb_data.category.at(pc_idx)};
            auto& category_proto{pb_proto.category(pc_idx)};

            category_data.property_type = category_proto.property_type();
            category_data.property = category_proto.property();
            category_data.property_conf = category_proto.property_conf();

            auto& str_value{category_proto.property_name()};
            memcpy(category_data.property_name, str_value.c_str(),
                   str_value.length());
          }  // end for category
        }    // end for - property_bbox

        for (std::int32_t kp_idx = 0; kp_idx < obst_proto.key_points_size();
             kp_idx++) {
          auto& key_point_data{obst_data.key_points.at(kp_idx)};
          auto& key_point_proto{obst_proto.key_points(kp_idx)};
          if (key_point_proto.has_pt()) {
            key_point_data.pt.x = key_point_proto.pt().x();
            key_point_data.pt.y = key_point_proto.pt().y();
            key_point_data.pt.z = key_point_proto.pt().z();
          }  // end pt
          key_point_data.type = key_point_proto.type();
          key_point_data.conf = key_point_proto.conf();
          key_point_data.boundary_conf_x = key_point_proto.boundary_conf_x();
          key_point_data.boundary_conf_y = key_point_proto.boundary_conf_y();
        }  // end for - key_points

        obst_data.depth = obst_proto.depth();
        obst_data.expand = obst_proto.expand();

        if (obst_proto.has_input_roi()) {
          obst_data.input_roi.bottom = obst_proto.input_roi().bottom();
          obst_data.input_roi.right = obst_proto.input_roi().right();
          obst_data.input_roi.left = obst_proto.input_roi().left();
          obst_data.input_roi.top = obst_proto.input_roi().top();
        }  // end for input_roi
      }    // end for - obstacle_raws

      obst_raw_data.num_key_points_raws = obst_raw_proto.key_points_raws_size();
      if (obst_raw_data.num_key_points_raws > kStageOneKeyPointsSize) {
        throw OutOfResourceException("Cylinder multitask data \
              -> key_points_raws info size too long");
      }
      for (std::uint8_t kp_raw_idx = 0; kp_raw_idx < obst_raw_data.num_key_points_raws;
           kp_raw_idx++) {
        auto& kp_raw{obst_raw_data.key_points_raws.at(kp_raw_idx)};
        auto& kp_raw_proto{obst_raw_proto.key_points_raws(kp_raw_idx)};

        kp_raw.type = static_cast<KeyPointType>(kp_raw_proto.type());
        kp_raw.num_key_points = kp_raw_proto.key_points_size();
        if (kp_raw.num_key_points > kMaxKeyPoints) {
          throw OutOfResourceException("Cylinder multitask data \
                -> key_points info size too long");
        }
        for (std::int32_t kp_idx = 0; kp_idx < kp_raw_proto.key_points_size();
             kp_idx++) {
          auto& key_point_data{kp_raw.key_points.at(kp_idx)};
          auto& key_point_proto{kp_raw_proto.key_points(kp_idx)};
          if (key_point_proto.has_pt()) {
            key_point_data.pt.x = key_point_proto.pt().x();
            key_point_data.pt.y = key_point_proto.pt().y();
            key_point_data.pt.z = key_point_proto.pt().z();
          }  // end pt
          key_point_data.type = key_point_proto.type();
          key_point_data.conf = key_point_proto.conf();
          key_point_data.boundary_conf_x = key_point_proto.boundary_conf_x();
          key_point_data.boundary_conf_y = key_point_proto.boundary_conf_y();
        }  // end for - key_points
      }
    }      // end obstacle_raw_data
  }

  std::uint8_t num_real3d_data =
              static_cast<std::uint8_t>(proto_msg->real3d_data_size());
  for (std::uint8_t idx = 0; ((idx < num_real3d_data) &&
                              (idx < CylinderMultiTaskData::kMaxCameraNumber));
       idx++) {
    {
      auto& real3d_data{cylinder_multitask_data.real3d_data.at(idx)};
      auto& real3d_proto{proto_msg->real3d_data(idx)};

      if (real3d_proto.has_header()) {
        real3d_data.header.cam_idx = real3d_proto.header().cam_idx();
        real3d_data.header.frame_id = real3d_proto.header().frame_id();
        real3d_data.header.time_stamp = real3d_proto.header().time_stamp();
      }

      for (std::int32_t rr_idx = 0; rr_idx < real3d_proto.real3d_results_size();
           rr_idx++) {
        auto& real3d_result_data{real3d_data.real3d_results.at(rr_idx)};
        auto& real3d_result_proto{real3d_proto.real3d_results(rr_idx)};

        if (real3d_result_proto.has_header()) {
          real3d_result_data.header.cam_idx =
              real3d_result_proto.header().cam_idx();
          real3d_result_data.header.frame_id =
              real3d_result_proto.header().frame_id();
          real3d_result_data.header.time_stamp =
              real3d_result_proto.header().time_stamp();
        }  // end header

        real3d_result_data.type =
            static_cast<Real3DType>(real3d_result_proto.type());

        auto& str_value{real3d_result_proto.coordinate_type()};
        real3d_result_data.num_coordinate_type = str_value.length();
        memcpy(real3d_result_data.coordinate_type.data(), str_value.c_str(),
               str_value.length());

        real3d_result_data.num_real3d_result =
            static_cast<std::uint8_t>(real3d_result_proto.real3d_result_size());
        for (std::uint8_t rr_idx = 0;
             rr_idx < real3d_result_data.num_real3d_result; rr_idx++) {
          auto& result_data{real3d_result_data.real3d_result.at(rr_idx)};
          auto& result_proto{real3d_result_proto.real3d_result(rr_idx)};

          if (result_proto.has_bev_obstacle()) {
            auto& bev_obst_data{result_data.bev_obstacle};
            auto& bev_obst_proto{result_proto.bev_obstacle()};

            DynamicObstacleDeserialize(bev_obst_proto, bev_obst_data);
          }    // end bev_obstacle

          if (result_proto.has_img_3dbox()) {
            auto& img_3dbox_data{result_data.img_3dbox};
            auto& img_3dbox_proto{result_proto.img_3dbox()};

            if (img_3dbox_proto.has_lower_lt()) {
              img_3dbox_data.lower_lt.x = img_3dbox_proto.lower_lt().x();
              img_3dbox_data.lower_lt.y = img_3dbox_proto.lower_lt().y();
              img_3dbox_data.lower_lt.z = img_3dbox_proto.lower_lt().z();
            }  // end lower_lt

            if (img_3dbox_proto.has_lower_lb()) {
              img_3dbox_data.lower_lb.x = img_3dbox_proto.lower_lb().x();
              img_3dbox_data.lower_lb.y = img_3dbox_proto.lower_lb().y();
              img_3dbox_data.lower_lb.z = img_3dbox_proto.lower_lb().z();
            }  // end lower_lb

            if (img_3dbox_proto.has_lower_rb()) {
              img_3dbox_data.lower_rb.x = img_3dbox_proto.lower_rb().x();
              img_3dbox_data.lower_rb.y = img_3dbox_proto.lower_rb().y();
              img_3dbox_data.lower_rb.z = img_3dbox_proto.lower_rb().z();
            }  // end lower_rb

            if (img_3dbox_proto.has_lower_rt()) {
              img_3dbox_data.lower_rt.x = img_3dbox_proto.lower_rt().x();
              img_3dbox_data.lower_rt.y = img_3dbox_proto.lower_rt().y();
              img_3dbox_data.lower_rt.z = img_3dbox_proto.lower_rt().z();
            }  // end lower_rt

            if (img_3dbox_proto.has_upper_lt()) {
              img_3dbox_data.upper_lt.x = img_3dbox_proto.upper_lt().x();
              img_3dbox_data.upper_lt.y = img_3dbox_proto.upper_lt().y();
              img_3dbox_data.upper_lt.z = img_3dbox_proto.upper_lt().z();
            }  // end upper_lt

            if (img_3dbox_proto.has_upper_lb()) {
              img_3dbox_data.upper_lb.x = img_3dbox_proto.upper_lb().x();
              img_3dbox_data.upper_lb.y = img_3dbox_proto.upper_lb().y();
              img_3dbox_data.upper_lb.z = img_3dbox_proto.upper_lb().z();
            }  // end upper_lb

            if (img_3dbox_proto.has_upper_rb()) {
              img_3dbox_data.upper_rb.x = img_3dbox_proto.upper_rb().x();
              img_3dbox_data.upper_rb.y = img_3dbox_proto.upper_rb().y();
              img_3dbox_data.upper_rb.z = img_3dbox_proto.upper_rb().z();
            }  // end upper_rb

            if (img_3dbox_proto.has_upper_rt()) {
              img_3dbox_data.upper_rt.x = img_3dbox_proto.upper_rt().x();
              img_3dbox_data.upper_rt.y = img_3dbox_proto.upper_rt().y();
              img_3dbox_data.upper_rt.z = img_3dbox_proto.upper_rt().z();
            }  // end upper_rt

            img_3dbox_data.conf = img_3dbox_proto.conf();
          }  // end img_3dbox

          if (result_proto.has_img_rect()) {
            result_data.img_rect.bottom = result_proto.img_rect().bottom();
            result_data.img_rect.right = result_proto.img_rect().right();
            result_data.img_rect.left = result_proto.img_rect().left();
            result_data.img_rect.top = result_proto.img_rect().top();
          }  // end img_rect
        }    // end for - real3d_result
      }      // end for - real3d_results
    }        // end real3d_data
  }

  return 0;
}

}  // namespace autodrive
