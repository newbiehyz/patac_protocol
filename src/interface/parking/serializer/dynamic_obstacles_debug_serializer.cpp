//

#include "interface/parking/serializer/dynamic_obstacles_debug_serializer.h"

#include <utils/exception.h>

#include "interface/parking/vision_debug_data.h"
#include "protocol/vision_debug.pb.h"
#include "serializer/common_serializer.h"
#include "type/serializer/dynamic_obstacle_debug_info_serializer.h"

namespace autodrive {

  int VisionDebugSerializer::Serialize(
      const message::SerializeMsg *data_ptr,
      message::DeserializeMsg *msg_ptr) const
  {
    const auto proto_msg = dynamic_cast<DMsgType *>(msg_ptr)->MutableMsg();
    const auto data_msg = dynamic_cast<const SMsgType *>(data_ptr)->GetData();
    CommonSerializer::Serialize(data_msg, proto_msg->mutable_header());
    auto &dyna_obsts_debug_data{data_msg->GetDynamicObstaclesDebugInfo()};

    for (std::int32_t obs_idx = 0; obs_idx < dyna_obsts_debug_data.num_obstacles;
         ++obs_idx)
    {
      workflow::proto::DynamicObstacleDebugInfo *dynaic_obst_debug_proto =
          proto_msg->add_obstacles_debug_info();
      auto &dynamic_obst_debug_data{dyna_obsts_debug_data.obstacles_debug_info.at(obs_idx)};

      DynamicObstacleDebugInfoSerialize(dynamic_obst_debug_data, dynaic_obst_debug_proto);
    } // end for
    return 0;
  }

  int VisionDebugSerializer::Deserialize(
      const message::DeserializeMsg *msg_ptr,
      message::SerializeMsg *data_ptr) const
  {
    const auto &proto_msg = dynamic_cast<const DMsgType *>(msg_ptr)->GetMsg();
    const auto &data_msg = dynamic_cast<SMsgType *>(data_ptr)->MutableData();
    CommonSerializer::Deserialize(proto_msg->mutable_header(), data_msg);
    auto &dynamic_obsts_debug_data{data_msg->MutableDynamicObstaclesDebugInfo()};

    dynamic_obsts_debug_data.num_obstacles = proto_msg->obstacles_debug_info_size();

    if (static_cast<std::uint32_t>(dynamic_obsts_debug_data.num_obstacles) >
        VisionDebugData::kMaxDynamicObstacleSize)
    {
      throw OutOfResourceException("Dynamic obstcles debug data info size too long");
    }

    for (std::int32_t obs_idx = 0; obs_idx < dynamic_obsts_debug_data.num_obstacles;
         obs_idx++)
    {
      auto &obst_debug_proto{proto_msg->obstacles_debug_info(obs_idx)};
      auto &obst_debug_data{dynamic_obsts_debug_data.obstacles_debug_info.at(obs_idx)};

      DynamicObstacleDebugInfoDeserialize(obst_debug_proto, obst_debug_data);
    }
    return 0;
  }

}  // namespace autodrive