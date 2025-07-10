//
#include "interface/parking/serializer/fail_safe_result_serializer.h"

#include <utils/exception.h>

#include "interface/parking/fail_safe_result_data.h"
#include "protocol/fail_safe_result.pb.h"
#include "protocol/perception_base.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int FailsafeResultSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto &failsaferesults_data{data_msg->GetFailSafeResults()};
  workflow::proto::FailSafeResults *failsaferesults_proto{proto_msg->mutable_safe_results()};
  for (std::uint32_t fs_idx = 0; fs_idx < failsaferesults_data.num_fail_safe_result; fs_idx++){
      auto &failsafe_result_data{failsaferesults_data.fail_safe_result_data.at(fs_idx)};
      auto &failsafelev_data{failsafe_result_data.fail_safe_result};
      workflow::proto::FailSafeResult *failsafe_result_proto = failsaferesults_proto->add_fail_safe_result_data();
      failsafe_result_proto->set_image_id(static_cast<workflow::proto::perception_base::ImageId>(failsafe_result_data.image_id));

      workflow::proto::FailSafeLevel *failsafelev_proto =
          failsafe_result_proto->mutable_fail_safe_result();
      failsafelev_proto->set_rain_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.rain_severity_level));
      failsafelev_proto->set_full_blockage_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.full_blockage_severity_level));
      failsafelev_proto->set_blurred_image_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.blurred_image_severity_level));
      failsafelev_proto->set_splashes_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.splashes_severity_level));
      failsafelev_proto->set_low_sun_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.low_sun_severity_level));
      failsafelev_proto->set_sun_ray_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.sun_ray_severity_level));

      failsafelev_proto->set_out_of_focus_severity_level(
          failsafelev_data.out_of_focus_severity_level);
      failsafelev_proto->set_out_of_calibration_severity_level(
          failsafelev_data.out_of_calibration_severity_level);

      failsafelev_proto->set_partial_blockage_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.partial_blockage_severity_level));
      failsafelev_proto->set_frozen_windshield_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.frozen_windshield_severity_level));
      failsafelev_proto->set_fog_severity_level(
          static_cast<workflow::proto::SeverityLevel>(
              failsafelev_data.fog_severity_level));

      failsafelev_proto->set_free_sight(failsafelev_data.free_sight);
      failsafelev_proto->set_out_of_calib_tsr_bs(
          static_cast<workflow::proto::OutOfCalibTsrBs>(
              failsafelev_data.out_of_calib_tsr_bs));
      // SceneParsingResults
      auto &scene_parsing_data{failsafe_result_data.scene};
      workflow::proto::perception_base::SceneParsing *scene_parsing_proto =
          failsafe_result_proto->mutable_scene();
      scene_parsing_proto->set_image_id(
          static_cast<workflow::proto::perception_base::ImageId>(scene_parsing_data.image_id));
      scene_parsing_proto->set_region_type(
          static_cast<workflow::proto::perception_base::SceneRegionType>(
              scene_parsing_data.region_type));
      scene_parsing_proto->set_illu_type(
          static_cast<workflow::proto::perception_base::SceneIlluminationType>(
              scene_parsing_data.illu_type));
      scene_parsing_proto->set_parking_type(
          static_cast<workflow::proto::perception_base::SceneParkingType>(
              scene_parsing_data.parking_type));
      scene_parsing_proto->set_region_conf(scene_parsing_data.region_conf);
      scene_parsing_proto->set_illu_conf(scene_parsing_data.illu_conf);
      scene_parsing_proto->set_parking_conf(scene_parsing_data.parking_conf);
  }
  return 0;
}

int FailsafeResultSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& failsaferesults_data{data_msg->MutableFailSafeResults()};
  if(proto_msg->has_safe_results()){
    auto& failsafe_results_proto{proto_msg->safe_results()};
    failsaferesults_data.num_fail_safe_result = failsafe_results_proto.fail_safe_result_data_size();
    for(std::uint32_t idx = 0; idx < failsaferesults_data.num_fail_safe_result; idx++){
        auto &failsafe_result_proto{failsafe_results_proto.fail_safe_result_data(idx)};
        auto &failsafe_result_data{failsaferesults_data.fail_safe_result_data.at(idx)};
        auto &failsafelev_proto{failsafe_result_proto.fail_safe_result()};
        auto &failsafelev_data{failsafe_result_data.fail_safe_result};
        failsafe_result_data.image_id = static_cast<autodrive::ImageId>(failsafe_result_proto.image_id());
        failsafelev_data.rain_severity_level =
            static_cast<FailsafeResultData::SeverityLevel>(
                failsafelev_proto.rain_severity_level());
        failsafelev_data.full_blockage_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.full_blockage_severity_level());
        failsafelev_data.blurred_image_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.blurred_image_severity_level());
        failsafelev_data.splashes_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.splashes_severity_level());
        failsafelev_data.low_sun_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.low_sun_severity_level());
        failsafelev_data.sun_ray_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.sun_ray_severity_level());

        failsafelev_data.out_of_focus_severity_level =
            failsafelev_proto.out_of_focus_severity_level();
        failsafelev_data.out_of_calibration_severity_level =
            failsafelev_proto.out_of_calibration_severity_level();

        failsafelev_data.partial_blockage_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.partial_blockage_severity_level());
        failsafelev_data.frozen_windshield_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.frozen_windshield_severity_level());
        failsafelev_data.fog_severity_level =
            static_cast<autodrive::FailsafeResultData::SeverityLevel>(
                failsafelev_proto.fog_severity_level());

        failsafelev_data.free_sight = failsafelev_proto.free_sight();

        failsafelev_data.out_of_calib_tsr_bs =
            static_cast<autodrive::FailsafeResultData::OutOfCalibTsrBs>(
                failsafelev_proto.out_of_calib_tsr_bs());
        auto &scene_parsing_data{failsafe_result_data.scene};
        auto &scene_parsing_proto{failsafe_result_proto.scene()};
        scene_parsing_data.image_id = static_cast<autodrive::ImageId>(
               scene_parsing_proto.image_id());
        scene_parsing_data.region_type =
            static_cast<autodrive::SceneRegionType>(
                scene_parsing_proto.region_type());
        scene_parsing_data.illu_type =
            static_cast<autodrive::SceneIlluminationType>(
                scene_parsing_proto.illu_type());
        scene_parsing_data.parking_type =
            static_cast<autodrive::SceneParkingType>(
                scene_parsing_proto.parking_type());
        scene_parsing_data.region_conf = scene_parsing_proto.region_conf();
        scene_parsing_data.illu_conf = scene_parsing_proto.illu_conf();
        scene_parsing_data.parking_conf = scene_parsing_proto.parking_conf();
    }
  }
  

  

  return 0;
}

}  // namespace autodrive