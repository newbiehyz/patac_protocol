//

#ifndef AUTODRIVE_TYPE_SERIALIZER_MAPINFO_HEADER_SERIALIZE_SERIALIZER_H_
#define AUTODRIVE_TYPE_SERIALIZER_MAPINFO_HEADER_SERIALIZE_SERIALIZER_H_

#include "interface/parking/map_display_data.h"
#include "protocol/map_display.pb.h"

namespace autodrive {
int MapInfoSerialize(const MapDisplayData::MapInfo& map_info_data,
                     workflow::proto::MapInfo* map_info_proto);
int MapCommonHeaderSerialize(
    const MapDisplayData::MapCommonHeader& map_header_data,
    workflow::proto::MapCommonHeader* map_header_proto);
int MapInfoDeserialize(const workflow::proto::MapInfo& map_info_proto,
                       MapDisplayData::MapInfo& map_info_data);
int MapCommonHeaderDeserialize(
    const workflow::proto::MapCommonHeader& map_header_proto,
    MapDisplayData::MapCommonHeader& map_header_data);

}  // namespace autodrive

#endif  // AUTODRIVE_TYPE_SERIALIZER_MAPINFO_HEADER_SERIALIZE_SERIALIZER_H_