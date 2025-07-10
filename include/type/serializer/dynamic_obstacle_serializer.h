//

#ifndef AUTODRIVE_TYPE_SERIALIZER_DYNAMIC_OBSTACLE_SERIALIZER_H_
#define AUTODRIVE_TYPE_SERIALIZER_DYNAMIC_OBSTACLE_SERIALIZER_H_

#include "protocol/dynamic_obstacle.pb.h"
#include "type/dynamic_obstacle.h"

namespace autodrive {

int DynamicObstacleSerialize(
    const DynamicObstacle& dynamic_obs_data,
    workflow::proto::DynamicObstacle* dynamic_obs_proto);

int DynamicObstacleDeserialize(
    const workflow::proto::DynamicObstacle& dynamic_obs_proto,
    DynamicObstacle& dynamic_obs_data);

}  // namespace autodrive

#endif  // AUTODRIVE_TYPE_SERIALIZER_DYNAMIC_OBSTACLE_SERIALIZER_H_