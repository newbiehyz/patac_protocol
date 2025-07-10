//
#include "model/serializer/calibration_result_serializer.h"
#include "serializer/common_serializer.h"
#include <utils/exception.h>
#include "model/calibration_result_data.h"
#include "protocol/calibration_result.pb.h"

namespace autodrive {

int CalibrationResultSerializer::Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto &calib_data{data_msg->GetCalibration()};
  proto_msg->set_real_camera_amount(calib_data.real_camera_amount);

  for (uint32_t i = 0; i < calib_data.num_results; ++i) {
    auto *result_msg = proto_msg->add_results();
    result_msg->set_cam_id(static_cast<workflow::proto::CameraID>(
        calib_data.results.at(i).cam_id));
    result_msg->set_calib_status(static_cast<workflow::proto::CalibStatus>(
        calib_data.results.at(i).calib_status));
    result_msg->set_progress_rate(calib_data.results.at(i).progress_rate);
    auto *extrinsic{result_msg->mutable_extrinsic()};
    extrinsic->set_x(calib_data.results.at(i).extrinsic.x);
    extrinsic->set_y(calib_data.results.at(i).extrinsic.y);
    extrinsic->set_z(calib_data.results.at(i).extrinsic.z);
    extrinsic->set_roll(calib_data.results.at(i).extrinsic.roll);
    extrinsic->set_pitch(calib_data.results.at(i).extrinsic.pitch);
    extrinsic->set_yaw(calib_data.results.at(i).extrinsic.yaw);
    extrinsic->set_num_extrinsic(
        calib_data.results.at(i).extrinsic.num_extrinsic);
    extrinsic->set_cam_id(static_cast<workflow::proto::CameraID>(
        calib_data.results.at(i).extrinsic.cam_id));
  }

  return 0;
}

int CalibrationResultSerializer::Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const {
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& calibration{data_msg->MutableCalibration()};

  calibration.real_camera_amount = proto_msg->real_camera_amount();
  if (static_cast<std::uint32_t>(proto_msg->results_size()) >
      CalibrationResultData::kMaxCameraSize) {
    throw OutOfResourceException("Camera size too long");
  }
  calibration.num_results =
      static_cast<std::uint8_t>(proto_msg->results_size());
  for (uint32_t i = 0; i < calibration.num_results; ++i) {
    const auto &result_msg = proto_msg->results(i);

    calibration.results.at(i).cam_id =
        static_cast<CameraID>(result_msg.cam_id());
    calibration.results.at(i).calib_status =
        static_cast<CalibrationResultData::CalibStatus>(
            result_msg.calib_status());
    calibration.results.at(i).progress_rate = result_msg.progress_rate();
    auto &extrinsic{calibration.results.at(i).extrinsic};
    extrinsic.x = result_msg.extrinsic().x();
    extrinsic.y = result_msg.extrinsic().y();
    extrinsic.z = result_msg.extrinsic().z();
    extrinsic.roll = result_msg.extrinsic().roll();
    extrinsic.pitch = result_msg.extrinsic().pitch();
    extrinsic.yaw = result_msg.extrinsic().yaw();
    extrinsic.num_extrinsic = result_msg.extrinsic().num_extrinsic();
    extrinsic.cam_id = static_cast<CameraID>(result_msg.extrinsic().cam_id());
  }

  return 0;
}

}