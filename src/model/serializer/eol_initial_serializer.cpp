//
#include "model/serializer/eol_initial_serializer.h"

#include <utils/exception.h>

#include "model/eol_initial_data.h"
#include "protocol/eol_initial.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int EolInitialSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());

  const auto& initial_data{data_msg->GetInitialData()};

  {
    proto_msg->mutable_vehicle_data()->set_wheelbase(
        initial_data.vehicle_data.wheelbase);
    for (uint32_t i = 0; i < initial_data.vehicle_data.num_ori_extrinsic; ++i) {
      workflow::proto::Extrinsic* ori_ex =
          proto_msg->mutable_vehicle_data()->add_ori_extrinsic();
      ori_ex->set_x(initial_data.vehicle_data.ori_extrinsic.at(i).x);
      ori_ex->set_y(initial_data.vehicle_data.ori_extrinsic.at(i).y);
      ori_ex->set_z(initial_data.vehicle_data.ori_extrinsic.at(i).z);
      ori_ex->set_roll(initial_data.vehicle_data.ori_extrinsic.at(i).roll);
      ori_ex->set_pitch(initial_data.vehicle_data.ori_extrinsic.at(i).pitch);
      ori_ex->set_yaw(initial_data.vehicle_data.ori_extrinsic.at(i).yaw);
      ori_ex->set_num_extrinsic(
          initial_data.vehicle_data.ori_extrinsic.at(i).num_extrinsic);
      ori_ex->set_cam_id(static_cast<workflow::proto::CameraID>(
          initial_data.vehicle_data.ori_extrinsic.at(i).cam_id));
    }
  }  // end vehicle_data

  {
    for (uint32_t i = 0; i < initial_data.num_cb_data; ++i) {
      workflow::proto::ChessboardData* cb_msg = proto_msg->add_cb_data();
      const auto& cb_data = initial_data.cb_data[i];
      cb_msg->set_camera_id(
          static_cast<workflow::proto::CameraID>(cb_data.camera_id));
      cb_msg->set_top_left_x(cb_data.top_left_x);
      cb_msg->set_top_left_y(cb_data.top_left_y);
      cb_msg->set_height_interval(cb_data.height_interval);
      cb_msg->set_width_interval(cb_data.width_interval);
    }
  }  // end cb_data

  {
    for (uint32_t i = 0; i < initial_data.num_cg_data; ++i) {
      workflow::proto::CircleGridData* cg_msg = proto_msg->add_cg_data();
      const auto& cg_data = initial_data.cg_data[i];
      cg_msg->set_camera_id(
          static_cast<workflow::proto::CameraID>(cg_data.camera_id));
      cg_msg->set_basic_pt_x(cg_data.basic_pt_x);
      cg_msg->set_basic_pt_y(cg_data.basic_pt_y);
      cg_msg->set_basic_pt_z(cg_data.basic_pt_z);
      cg_msg->set_height_interval(cg_data.height_interval);
      cg_msg->set_width_interval(cg_data.width_interval);
    }
  }  // end cg_data

  return 0;
}

int EolInitialSerializer::Deserialize(const message::DeserializeMsg* msg_ptr,
                                      message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& eol_initial_data{data_msg->MutableInitialData()};

  {
    eol_initial_data.vehicle_data.wheelbase =
        proto_msg->vehicle_data().wheelbase();
    eol_initial_data.vehicle_data.num_ori_extrinsic =
        proto_msg->vehicle_data().ori_extrinsic_size();
    if (static_cast<std::uint32_t>(
            proto_msg->vehicle_data().ori_extrinsic_size()) >
        EolInitialData::kMaxCameraSize) {
      throw OutOfResourceException("EolInitial ori extrinsic size too long");
    }
    for (uint32_t i = 0; i < eol_initial_data.vehicle_data.num_ori_extrinsic;
         ++i) {
      eol_initial_data.vehicle_data.ori_extrinsic[i].x =
          proto_msg->vehicle_data().ori_extrinsic(i).x();
      eol_initial_data.vehicle_data.ori_extrinsic[i].y =
          proto_msg->vehicle_data().ori_extrinsic(i).y();
      eol_initial_data.vehicle_data.ori_extrinsic[i].z =
          proto_msg->vehicle_data().ori_extrinsic(i).z();
      eol_initial_data.vehicle_data.ori_extrinsic[i].roll =
          proto_msg->vehicle_data().ori_extrinsic(i).roll();
      eol_initial_data.vehicle_data.ori_extrinsic[i].pitch =
          proto_msg->vehicle_data().ori_extrinsic(i).pitch();
      eol_initial_data.vehicle_data.ori_extrinsic[i].yaw =
          proto_msg->vehicle_data().ori_extrinsic(i).yaw();
      eol_initial_data.vehicle_data.ori_extrinsic[i].num_extrinsic =
          proto_msg->vehicle_data().ori_extrinsic(i).num_extrinsic();
      eol_initial_data.vehicle_data.ori_extrinsic[i].cam_id =
          static_cast<CameraID>(
              proto_msg->vehicle_data().ori_extrinsic(i).cam_id());
    }
  }  // end vehicle_data

  {
    eol_initial_data.num_cb_data = proto_msg->cb_data_size();
    if (static_cast<std::uint32_t>(proto_msg->cb_data_size()) >
        EolInitialData::kMaxSVCSize) {
      throw OutOfResourceException("EolInitial cb data size too long");
    }
    for (uint32_t i = 0; i < eol_initial_data.num_cb_data; ++i) {
      eol_initial_data.cb_data[i].camera_id =
          static_cast<CameraID>(proto_msg->cb_data(i).camera_id());
      eol_initial_data.cb_data[i].top_left_x =
          proto_msg->cb_data(i).top_left_x();
      eol_initial_data.cb_data[i].top_left_y =
          proto_msg->cb_data(i).top_left_y();
      eol_initial_data.cb_data[i].height_interval =
          proto_msg->cb_data(i).height_interval();
      eol_initial_data.cb_data[i].width_interval =
          proto_msg->cb_data(i).width_interval();
    }
  }  // end cb_data

  {
    eol_initial_data.num_cg_data = proto_msg->cg_data_size();
    if (static_cast<std::uint32_t>(proto_msg->cg_data_size()) >
        EolInitialData::kMaxRLCSize) {
      throw OutOfResourceException("EolInitial cg data size too long");
    }
    for (uint32_t i = 0; i < eol_initial_data.num_cg_data; ++i) {
      eol_initial_data.cg_data[i].camera_id =
          static_cast<CameraID>(proto_msg->cg_data(i).camera_id());
      eol_initial_data.cg_data[i].basic_pt_x =
          proto_msg->cg_data(i).basic_pt_x();
      eol_initial_data.cg_data[i].basic_pt_y =
          proto_msg->cg_data(i).basic_pt_z();
      eol_initial_data.cg_data[i].basic_pt_y =
          proto_msg->cg_data(i).basic_pt_z();
      eol_initial_data.cg_data[i].height_interval =
          proto_msg->cg_data(i).height_interval();
      eol_initial_data.cg_data[i].width_interval =
          proto_msg->cg_data(i).width_interval();
    }
  }  // end cg_data

  return 0;
}

}  // namespace autodrive