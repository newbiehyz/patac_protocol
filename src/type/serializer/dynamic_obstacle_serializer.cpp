//

#include "type/serializer/dynamic_obstacle_serializer.h"

namespace autodrive {

int DynamicObstacleSerialize(
    const DynamicObstacle& dynamic_obst_data,
    workflow::proto::DynamicObstacle* dynaic_obst_proto) {
  dynaic_obst_proto->set_id(dynamic_obst_data.id);
  dynaic_obst_proto->set_life_time(dynamic_obst_data.life_time);
  dynaic_obst_proto->set_cam_id(dynamic_obst_data.cam_id);
  dynaic_obst_proto->set_type(static_cast<workflow::proto::DynamicObstacleType>(
      dynamic_obst_data.type));
  dynaic_obst_proto->set_sub_type(
      static_cast<workflow::proto::VehicleType>(dynamic_obst_data.sub_type));
  dynaic_obst_proto->set_time_stamp(dynamic_obst_data.time_stamp);
  dynaic_obst_proto->set_conf(dynamic_obst_data.conf);
  dynaic_obst_proto->set_age(dynamic_obst_data.age);

  {
    workflow::proto::perception_base::ImageSpace* image_space_proto =
        dynaic_obst_proto->mutable_img_info();
    auto& image_space_data{dynamic_obst_data.img_info};
    image_space_proto->mutable_rect()->set_bottom(image_space_data.rect.bottom);
    image_space_proto->mutable_rect()->set_left(image_space_data.rect.left);
    image_space_proto->mutable_rect()->set_right(image_space_data.rect.right);
    image_space_proto->mutable_rect()->set_top(image_space_data.rect.top);
  }  // end img_info

  {
    workflow::proto::DynamicObstWorldSpaceInfo* world_info_proto =
        dynaic_obst_proto->mutable_world_info();
    auto& world_info_data{dynamic_obst_data.world_info};
    world_info_proto->set_yaw(world_info_data.yaw);
    world_info_proto->set_length(world_info_data.length);
    world_info_proto->set_width(world_info_data.width);
    world_info_proto->set_height(world_info_data.height);
    world_info_proto->set_traversable(world_info_data.traversable);
    {
      auto& polygon_data{world_info_data.poly};
      workflow::proto::perception_base::Polygon* polygon_proto =
          world_info_proto->mutable_poly();
      for (std::uint32_t p_idx = 0; p_idx < polygon_data.num_points; p_idx++) {
        auto& point_data{polygon_data.points.at(p_idx)};
        workflow::proto::Vector3f* point_proto =
            polygon_proto->add_polygon_points();
        point_proto->set_x(point_data.x);
        point_proto->set_y(point_data.y);
        point_proto->set_z(point_data.z);
      }
    }  // end poly

    {
      auto& polygon_vcs_data{world_info_data.poly_vcs};
      workflow::proto::perception_base::Polygon* polygon_proto =
          world_info_proto->mutable_poly_vcs();
      for (std::uint32_t p_idx = 0; p_idx < polygon_vcs_data.num_points;
           p_idx++) {
        auto& point_data{polygon_vcs_data.points.at(p_idx)};
        workflow::proto::Vector3f* point_proto =
            polygon_proto->add_polygon_points();
        point_proto->set_x(point_data.x);
        point_proto->set_y(point_data.y);
        point_proto->set_z(point_data.z);
      }
    }  // end poly_vcs

    world_info_proto->mutable_position()->set_x(world_info_data.position.x);
    world_info_proto->mutable_position()->set_y(world_info_data.position.y);
    world_info_proto->mutable_position()->set_z(world_info_data.position.z);

    world_info_proto->mutable_rbc_position()->set_x(
        world_info_data.rbc_position.x);
    world_info_proto->mutable_rbc_position()->set_y(
        world_info_data.rbc_position.y);
    world_info_proto->mutable_rbc_position()->set_z(
        world_info_data.rbc_position.z);

    world_info_proto->set_ttc(world_info_data.ttc);

    world_info_proto->mutable_vel()->set_vx(world_info_data.vel.vx);
    world_info_proto->mutable_vel()->set_vy(world_info_data.vel.vy);
    world_info_proto->mutable_vel()->set_vz(world_info_data.vel.vz);

    world_info_proto->mutable_vel_abs_world()->set_vx(
        world_info_data.vel_abs_world.vx);
    world_info_proto->mutable_vel_abs_world()->set_vy(
        world_info_data.vel_abs_world.vy);
    world_info_proto->mutable_vel_abs_world()->set_vz(
        world_info_data.vel_abs_world.vz);

    world_info_proto->mutable_acc_abs_world()->set_ax(
        world_info_data.acc_abs_world.ax);
    world_info_proto->mutable_acc_abs_world()->set_ay(
        world_info_data.acc_abs_world.ay);
    world_info_proto->mutable_acc_abs_world()->set_az(
        world_info_data.acc_abs_world.az);

    world_info_proto->mutable_acc()->set_ax(
        world_info_data.acc.ax);
    world_info_proto->mutable_acc()->set_ay(
        world_info_data.acc.ay);
    world_info_proto->mutable_acc()->set_az(
        world_info_data.acc.az);

    world_info_proto->set_motion_status(
        static_cast<workflow::proto::DynamicObstacleMotionStatus>(
            world_info_data.motion_status));
    world_info_proto->set_motion_category(
        static_cast<workflow::proto::MotionCategory>(
            world_info_data.motion_category));
    world_info_proto->set_position_type(
        static_cast<workflow::proto::OutputObsPositionType>(
            world_info_data.position_type));
    world_info_proto->set_visible_side(
        static_cast<workflow::proto::VisibleSide>(
            world_info_data.visible_side));
    world_info_proto->set_visible_rect(
        static_cast<workflow::proto::VisibleRect>(
            world_info_data.visible_rect));

    world_info_proto->set_yaw_rate(world_info_data.yaw_rate);
    world_info_proto->set_motion_orientation(
        static_cast<workflow::proto::MotionOrientation>(
            world_info_data.motion_orientation));

    for (size_t sv_idx = 0; sv_idx < world_info_data.sigma_vel.size();
         sv_idx++) {
      world_info_proto->add_sigma_vel(world_info_data.sigma_vel.at(sv_idx));
    }

    world_info_proto->set_sigma_yaw(world_info_data.sigma_yaw);
    world_info_proto->set_sigma_width(world_info_data.sigma_width);
    world_info_proto->set_sigma_height(world_info_data.sigma_height);
    world_info_proto->set_sigma_length(world_info_data.sigma_length);

    for (size_t sp_idx = 0; sp_idx < world_info_data.sigma_position.size();
         sp_idx++) {
      world_info_proto->add_sigma_position(
          world_info_data.sigma_position.at(sp_idx));
    }

    for (size_t sp_idx = 0; sp_idx < world_info_data.sigma_rbc_position.size();
         sp_idx++) {
      world_info_proto->add_sigma_rbc_position(
          world_info_data.sigma_rbc_position.at(sp_idx));
    }

    world_info_proto->set_sigma_yaw_rate(world_info_data.sigma_yaw_rate);
    world_info_proto->set_head_rear_type(world_info_data.head_rear_type);

    for (size_t sc_idx = 0; sc_idx < world_info_data.sigma_acc_ref.size();
         sc_idx++) {
      world_info_proto->add_sigma_acc_ref(
          world_info_data.sigma_acc_ref.at(sc_idx));
    }

    {
      workflow::proto::ObstacleAngle* obj_angle_proto =
          world_info_proto->mutable_obj_angle();
      auto& obj_angle_data{world_info_data.obj_angle};
      obj_angle_proto->set_angle_left_v(obj_angle_data.angle_left_v);
      obj_angle_proto->set_angle_left(obj_angle_data.angle_left);
      obj_angle_proto->set_angle_middle_v(obj_angle_data.angle_middle_v);
      obj_angle_proto->set_angle_middle(obj_angle_data.angle_middle);
      obj_angle_proto->set_angle_right_v(obj_angle_data.angle_right_v);
      obj_angle_proto->set_angle_right(obj_angle_data.angle_right);
      obj_angle_proto->set_angle_side_v(obj_angle_data.angle_side_v);
      obj_angle_proto->set_angle_side(obj_angle_data.angle_side);
    }  // end obj_angle
    world_info_proto->set_motion_orientation(
        static_cast<workflow::proto::MotionOrientation>(
            world_info_data.motion_orientation));
    world_info_proto->set_lane_assignment(
        static_cast<std::int32_t>(world_info_data.lane_assignment));
    world_info_proto->set_cipv(static_cast<std::int32_t>(world_info_data.cipv));
  }  // end world_info

  dynaic_obst_proto->set_valid_info(dynamic_obst_data.valid_info);
  dynaic_obst_proto->set_select_level(dynamic_obst_data.select_level);

  for (size_t c_idx = 0; c_idx < dynamic_obst_data.num_categories; c_idx++) {
    workflow::proto::perception_base::Category* cate_proto =
        dynaic_obst_proto->add_category();
    auto& cate_data{dynamic_obst_data.categories.at(c_idx)};
    cate_proto->set_property_type(cate_data.property_type);
    cate_proto->set_property(cate_data.property);
    cate_proto->set_property_conf(cate_data.property_conf);
    cate_proto->set_property_name(cate_data.property_name);
  }

  return 0;
}

int DynamicObstacleDeserialize(
    const workflow::proto::DynamicObstacle& dynamic_obs_proto,
    DynamicObstacle& dynamic_obs_data) {
  dynamic_obs_data.id = dynamic_obs_proto.id();
  dynamic_obs_data.life_time = dynamic_obs_proto.life_time();
  dynamic_obs_data.cam_id = dynamic_obs_proto.cam_id();
  dynamic_obs_data.type = static_cast<ObstacleType>(dynamic_obs_proto.type());
  dynamic_obs_data.sub_type =
      static_cast<DynamicVehicleType>(dynamic_obs_proto.sub_type());
  dynamic_obs_data.time_stamp = dynamic_obs_proto.time_stamp();
  dynamic_obs_data.conf = dynamic_obs_proto.conf();
  dynamic_obs_data.age = dynamic_obs_proto.age();

  if (dynamic_obs_proto.has_img_info()) {
    auto& image_info_proto{dynamic_obs_proto.img_info()};
    dynamic_obs_data.img_info.rect.bottom = image_info_proto.rect().bottom();
    dynamic_obs_data.img_info.rect.left = image_info_proto.rect().left();
    dynamic_obs_data.img_info.rect.right = image_info_proto.rect().right();
    dynamic_obs_data.img_info.rect.top = image_info_proto.rect().top();
  }  // end img_info

  if (dynamic_obs_proto.has_world_info()) {
    auto& world_info_proto{dynamic_obs_proto.world_info()};
    auto& world_info_data{dynamic_obs_data.world_info};
    world_info_data.yaw = world_info_proto.yaw();
    world_info_data.length = world_info_proto.length();
    world_info_data.width = world_info_proto.width();
    world_info_data.height = world_info_proto.height();
    world_info_data.traversable = world_info_proto.traversable();
    world_info_data.vel.vx = world_info_proto.vel().vx();
    world_info_data.vel.vy = world_info_proto.vel().vy();
    world_info_data.vel.vz = world_info_proto.vel().vz();

    if (world_info_proto.has_poly())
    {
        auto &polygon_data{world_info_data.poly};
        auto &polygon_proto{world_info_proto.poly()};
        polygon_data.num_points = polygon_proto.polygon_points_size();
        for (std::uint32_t p_idx = 0; p_idx < polygon_data.num_points; p_idx++)
        {
            auto &point_data{polygon_data.points.at(p_idx)};
            auto &point_proto{polygon_proto.polygon_points(p_idx)};
            point_data.x = point_proto.x();
            point_data.y = point_proto.y();
            point_data.z = point_proto.z();
        }
    }

    if (world_info_proto.has_poly_vcs()) {
      auto& polygon_vcs_data{world_info_data.poly_vcs};
      auto& polygon_vcs_proto{world_info_proto.poly_vcs()};
      polygon_vcs_data.num_points = polygon_vcs_proto.polygon_points_size();
      for (std::uint32_t p_idx = 0; p_idx < polygon_vcs_data.num_points;
           p_idx++) {
        auto& point_data{polygon_vcs_data.points.at(p_idx)};
        auto& point_proto{polygon_vcs_proto.polygon_points(p_idx)};
        point_data.x = point_proto.x();
        point_data.y = point_proto.y();
        point_data.z = point_proto.z();
      }
    }

    world_info_data.position.x = world_info_proto.position().x();
    world_info_data.position.y = world_info_proto.position().y();
    world_info_data.position.z = world_info_proto.position().z();
    world_info_data.rbc_position.x = world_info_proto.rbc_position().x();
    world_info_data.rbc_position.y = world_info_proto.rbc_position().y();
    world_info_data.rbc_position.z = world_info_proto.rbc_position().z();
    world_info_data.ttc = world_info_proto.ttc();

    if (world_info_proto.has_vel_abs_world()) {
      world_info_data.vel_abs_world.vx = world_info_proto.vel_abs_world().vx();
      world_info_data.vel_abs_world.vy = world_info_proto.vel_abs_world().vy();
      world_info_data.vel_abs_world.vz = world_info_proto.vel_abs_world().vz();
    }  // end vel_abs_world

    if (world_info_proto.has_acc_abs_world()) {
      world_info_data.acc_abs_world.ax = world_info_proto.acc_abs_world().ax();
      world_info_data.acc_abs_world.ay = world_info_proto.acc_abs_world().ay();
      world_info_data.acc_abs_world.az = world_info_proto.acc_abs_world().az();
    }  // end acc_abs_world

    if (world_info_proto.has_acc()) {
      world_info_data.acc.ax = world_info_proto.acc().ax();
      world_info_data.acc.ay = world_info_proto.acc().ay();
      world_info_data.acc.az = world_info_proto.acc().az();
    }  // end acc

    world_info_data.motion_status =
        static_cast<MotionStatus>(world_info_proto.motion_status());
    world_info_data.motion_category =
        static_cast<MotionCategory>(world_info_proto.motion_category());
    world_info_data.position_type =
        static_cast<OutputObsPositionType>(world_info_proto.position_type());

    world_info_data.visible_side =
        static_cast<VisibleSide>(world_info_proto.visible_side());
    world_info_data.visible_rect =
        static_cast<VisibleRect>(world_info_proto.visible_rect());

    world_info_data.yaw_rate = world_info_proto.yaw_rate();
    world_info_data.motion_orientation =
        static_cast<MotionOrientation>(world_info_proto.motion_orientation());

    for (std::int32_t sv_idx = 0; sv_idx < world_info_proto.sigma_vel_size();
         sv_idx++) {
      world_info_data.sigma_vel.at(sv_idx) = world_info_proto.sigma_vel(sv_idx);
    }

    world_info_data.sigma_yaw = world_info_proto.sigma_yaw();
    world_info_data.sigma_width = world_info_proto.sigma_width();
    world_info_data.sigma_height = world_info_proto.sigma_height();
    world_info_data.sigma_length = world_info_proto.sigma_length();

    for (std::int32_t sp_idx = 0;
         sp_idx < world_info_proto.sigma_position_size(); sp_idx++) {
      world_info_data.sigma_position.at(sp_idx) =
          world_info_proto.sigma_position(sp_idx);
    }

    for (std::int32_t sp_idx = 0;
         sp_idx < world_info_proto.sigma_rbc_position_size(); sp_idx++) {
      world_info_data.sigma_rbc_position.at(sp_idx) =
          world_info_proto.sigma_rbc_position(sp_idx);
    }

    world_info_data.sigma_yaw_rate = world_info_proto.sigma_yaw_rate();
    world_info_data.head_rear_type = world_info_proto.head_rear_type();

    for (std::int32_t sr_idx = 0;
         sr_idx < world_info_proto.sigma_acc_ref_size(); sr_idx++) {
      world_info_data.sigma_acc_ref.at(sr_idx) =
          world_info_proto.sigma_acc_ref(sr_idx);
    }

    if (world_info_proto.has_obj_angle()) {
      auto& obj_angle_proto{world_info_proto.obj_angle()};
      auto& obj_angle_data{world_info_data.obj_angle};
      obj_angle_data.angle_left_v = obj_angle_proto.angle_left_v();
      obj_angle_data.angle_left = obj_angle_proto.angle_left();
      obj_angle_data.angle_middle_v = obj_angle_proto.angle_middle_v();
      obj_angle_data.angle_middle = obj_angle_proto.angle_middle();
      obj_angle_data.angle_right_v = obj_angle_proto.angle_right_v();
      obj_angle_data.angle_right = obj_angle_proto.angle_right();
      obj_angle_data.angle_side_v = obj_angle_proto.angle_side_v();
      obj_angle_data.angle_side = obj_angle_proto.angle_side();
    }  // end obs_angle

    world_info_data.motion_orientation =
        static_cast<MotionOrientation>(world_info_proto.motion_orientation());

    world_info_data.lane_assignment =
        static_cast<LaneAssignment>(world_info_proto.lane_assignment());
    world_info_data.cipv = world_info_proto.cipv();
  }  // end world info

  dynamic_obs_data.valid_info = dynamic_obs_proto.valid_info();
  dynamic_obs_data.select_level = dynamic_obs_proto.select_level();
  dynamic_obs_data.num_categories = dynamic_obs_proto.category_size();

  for (std::uint32_t c_idx = 0; c_idx < dynamic_obs_data.num_categories;
       c_idx++) {
    auto& category_proto{dynamic_obs_proto.category(c_idx)};
    auto& category_data{dynamic_obs_data.categories.at(c_idx)};
    category_data.property_type = category_proto.property_type();
    category_data.property = category_proto.property();
    category_data.property_conf = category_proto.property_conf();
    auto& pro_name{category_proto.property_name()};
    memcpy(&category_data.property_name, pro_name.c_str(), pro_name.length());
  }  // end Category

  return 0;
}

}  // namespace autodrive
