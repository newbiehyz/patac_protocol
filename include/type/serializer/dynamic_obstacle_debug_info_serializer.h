//

#ifndef AUTODRIVE_TYPE_SERIALIZER_DYNAMIC_OBSTACLE_DEBUG_INFO_SERIALIZER_H_
#define AUTODRIVE_TYPE_SERIALIZER_DYNAMIC_OBSTACLE_DEBUG_INFO_SERIALIZER_H_

#include "protocol/dynamic_obstacle_debug_info.pb.h"
#include "type/dynamic_obstacle_debug_info.h"

namespace autodrive
{

    int DynamicObstacleDebugInfoSerialize(
        const DynamicObstacleDebugInfo &dynamic_obs_debug_info,
        workflow::proto::DynamicObstacleDebugInfo *dynamic_obs_debug_proto);

    int DynamicObstacleDebugInfoDeserialize(
        const workflow::proto::DynamicObstacleDebugInfo &dynamic_obs_debug_proto,
        DynamicObstacleDebugInfo &dynamic_obs_debug_info);

} // namespace autodrive

#endif // AUTODRIVE_TYPE_SERIALIZER_DYNAMIC_OBSTACLE_DEBUG_INFO_SERIALIZER_H_