//
#ifndef AUTODRIVE_MODEL_SERIALIZER_GNSS_H_
#define AUTODRIVE_MODEL_SERIALIZER_GNSS_H_

#include <cstdint>
#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class GnssMsg;
}
}

class GnssData;

class GnssSerializer : public message::ProtobufSerializer<workflow::proto::GnssMsg, GnssData> {
public:
  GnssSerializer() noexcept = default;
  ~GnssSerializer() noexcept = default;
  GnssSerializer(GnssSerializer const&) = default;
  GnssSerializer& operator=(GnssSerializer const&) = default;
  GnssSerializer(GnssSerializer&&) = default;
  GnssSerializer& operator=(GnssSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;

private:
  static constexpr std::uint32_t kSecond2Ns{1000000000};
};

}

#endif