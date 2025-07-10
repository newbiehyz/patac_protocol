//

#include "interface/parking/serializer/dynamic_obstacles_gt_serializer.h"

#include <utils/exception.h>

#include "interface/parking/dynamic_obstacles_gt_data.h"
#include "protocol/dynamic_obstacles.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/dynamic_obstacle_serializer.h"

namespace autodrive {

int DynamicObstaclesGtSerializer::Serialize(
    const message::SerializeMsg* data_ptr,
    message::DeserializeMsg* msg_ptr) const {
  const auto proto_msg = dynamic_cast<DMsgType*>(msg_ptr)->MutableMsg();
  const auto data_msg = dynamic_cast<const SMsgType*>(data_ptr)->GetData();
  CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
  auto& dyna_obsts_data{data_msg->GetDynamicObstacles()};

  proto_msg->set_cipv_id(dyna_obsts_data.cipv_id);

  for (std::int32_t obs_idx = 0; obs_idx < dyna_obsts_data.num_obstacles;
       ++obs_idx) {
    workflow::proto::DynamicObstacle* dynaic_obst_proto =
        proto_msg->add_obstacles();
    auto& dynamic_obst_data{dyna_obsts_data.obstacles.at(obs_idx)};

    DynamicObstacleSerialize(dynamic_obst_data, dynaic_obst_proto);
  }  // end for
  return 0;
}

int DynamicObstaclesGtSerializer::Deserialize(
    const message::DeserializeMsg* msg_ptr,
    message::SerializeMsg* data_ptr) const {
  const auto& proto_msg = dynamic_cast<const DMsgType*>(msg_ptr)->GetMsg();
  const auto& data_msg = dynamic_cast<SMsgType*>(data_ptr)->MutableData();
  CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
  auto& dynamic_obsts_data{data_msg->MutableDynamicObstacles()};

  dynamic_obsts_data.cipv_id = proto_msg->cipv_id();

  dynamic_obsts_data.num_obstacles = proto_msg->obstacles_size();

  if (static_cast<std::uint32_t>(dynamic_obsts_data.num_obstacles) >
      DynamicObstaclesGtData::kMaxDynamicObstacleSize) {
    throw OutOfResourceException("Dynamic obstcles data info size too long");
  }

  for (std::int32_t obs_idx = 0; obs_idx < dynamic_obsts_data.num_obstacles;
       obs_idx++) {
    auto& obst_proto{proto_msg->obstacles(obs_idx)};
    auto& obst_data{dynamic_obsts_data.obstacles.at(obs_idx)};

    DynamicObstacleDeserialize(obst_proto, obst_data);
  }
  return 0;
}

}  // namespace autodrive