//
#include "interface/parking/serializer/iqa_parsing_serializer.h"

#include <utils/exception.h>

#include "interface/parking/iqa_parsing_data.h"
#include "protocol/iqa_parsing.pb.h"

#include "protocol/perception_base.pb.h"
#include "serializer/common_serializer.h"

namespace autodrive {

int IqaParsingSerializer::Serialize(const message::SerializeMsg* data_ptr,
                                    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto &iqa_parse_data{data_msg->GetIqaParsingResults()};
  workflow::proto::IqaParsingResults *iqa_parsing_data_ = proto_msg->mutable_iqa_parsing_data();
  for (std::uint32_t idx = 0; idx < iqa_parse_data.num_iqa_parsing_result; idx++)
  {
    auto &iqa_parsing_result_data{iqa_parse_data.iqa_parsing_result_data.at(idx)};
    auto &parsing_data{iqa_parsing_result_data.parsing_data};
    workflow::proto::IqaParsing *iqa_parsing_result_proto = iqa_parsing_data_->add_iqa_parsing_result_data();
    workflow::proto::CylinderParsingImageData *cylinder_parsing_image_data = iqa_parsing_result_proto->mutable_parsing_data();
    iqa_parsing_result_proto->set_image_id(static_cast<workflow::proto::perception_base::ImageId>(iqa_parsing_result_data.image_id));
    cylinder_parsing_image_data->set_data((char *)parsing_data.data,
                                          kMaxCylParsingDataSize);
    cylinder_parsing_image_data->set_width(parsing_data.width);
    cylinder_parsing_image_data->set_height(parsing_data.height);
    cylinder_parsing_image_data->set_channel(parsing_data.channel);
  }
  // SceneParsingResults
  auto &scene_parsing_data{data_msg->GetSceneParsingResults()};
  workflow::proto::SceneParsingResults *scene_parsing_proto =
      proto_msg->mutable_scene_parsing_data();
  if (scene_parsing_data.num_scene_parsing_result >
      IqaParsingData::kMaxCameraSize) {
    throw OutOfResourceException(
        "scene_parsing_data.num_scene_parsing_result size too long");
  }
  for (std::uint32_t idx = 0; idx < scene_parsing_data.num_scene_parsing_result;
       idx++) {
    auto &scene_parsing_result_data{
        scene_parsing_data.scene_parsing_result_data.at(idx)};
    workflow::proto::perception_base::SceneParsing *scene_parsing_result_proto =
        scene_parsing_proto->add_scene_parsing_result_data();
    scene_parsing_result_proto->set_image_id(
        static_cast<workflow::proto::perception_base::ImageId>(
            scene_parsing_result_data.image_id));
    scene_parsing_result_proto->set_region_type(
        static_cast<workflow::proto::perception_base::SceneRegionType>(
            scene_parsing_result_data.region_type));
    scene_parsing_result_proto->set_illu_type(
        static_cast<workflow::proto::perception_base::SceneIlluminationType>(
            scene_parsing_result_data.illu_type));
    scene_parsing_result_proto->set_parking_type(
        static_cast<workflow::proto::perception_base::SceneParkingType>(
            scene_parsing_result_data.parking_type));
    scene_parsing_result_proto->set_region_conf(
        scene_parsing_result_data.region_conf);
    scene_parsing_result_proto->set_illu_conf(
        scene_parsing_result_data.illu_conf);
    scene_parsing_result_proto->set_parking_conf(
        scene_parsing_result_data.parking_conf);
  }

  return 0;
}

int IqaParsingSerializer::Deserialize(const message::DeserializeMsg *msg_ptr,
                                      message::SerializeMsg *data_ptr) const
{
  const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
  const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  if (proto_msg->has_iqa_parsing_data())
  {
    auto &iqa_parsing_results_data{data_msg->MutableIqaParsingResults()};
    auto &iqa_parsing_results_proto{proto_msg->iqa_parsing_data()};
    iqa_parsing_results_data.num_iqa_parsing_result = iqa_parsing_results_proto.iqa_parsing_result_data_size();
    for (std::uint32_t idx = 0; idx < iqa_parsing_results_data.num_iqa_parsing_result; idx++)
    {
      auto &iqa_parsing_data{iqa_parsing_results_data.iqa_parsing_result_data.at(idx)};
      auto &iqa_parsing_proto{iqa_parsing_results_proto.iqa_parsing_result_data(idx)};
      auto &cylinder_parsing_image_data{iqa_parsing_proto.parsing_data()};
      auto &parsing_data{iqa_parsing_data.parsing_data};
      std::string str_value{cylinder_parsing_image_data.data()};
      iqa_parsing_data.image_id = static_cast<autodrive::ImageId>(iqa_parsing_proto.image_id());
      if (str_value.length() > kMaxCylParsingDataSize)
      {
        throw OutOfResourceException("iqa_parsing_data->data info size too long");
      }

      memcpy(parsing_data.data, str_value.c_str(), str_value.length());
      parsing_data.width = cylinder_parsing_image_data.width();
      parsing_data.height = cylinder_parsing_image_data.height();
      parsing_data.channel = cylinder_parsing_image_data.channel();
    }
  }

  if (proto_msg->has_scene_parsing_data()) {
    auto &scene_parsing_results_data{data_msg->MutableSceneParsingResults()};
    auto &scene_parsing_results_proto{proto_msg->scene_parsing_data()};
    scene_parsing_results_data.num_scene_parsing_result =
        scene_parsing_results_proto.scene_parsing_result_data_size();
    for (std::uint32_t idx = 0;
         idx < scene_parsing_results_data.num_scene_parsing_result; idx++) {
      auto &scene_parsing_data{
          scene_parsing_results_data.scene_parsing_result_data.at(idx)};
      auto &scene_parsing_proto{
          scene_parsing_results_proto.scene_parsing_result_data(idx)};
      scene_parsing_data.image_id =
          static_cast<autodrive::ImageId>(scene_parsing_proto.image_id());
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