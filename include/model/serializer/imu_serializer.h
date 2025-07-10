//
#ifndef AUTODRIVE_MODEL_SERIALIZER_IMU_H_
#define AUTODRIVE_MODEL_SERIALIZER_IMU_H_

#include <cstdint>
#include <foundation/message/proto/proto_serializer.h>

namespace autodrive {

namespace workflow {
namespace proto {
class ImuMsg;
}
}

class ImuData;

class ImuSerializer : public message::ProtobufSerializer<workflow::proto::ImuMsg, ImuData> {
public:
  ImuSerializer() noexcept = default;
  ~ImuSerializer() noexcept = default;
  ImuSerializer(ImuSerializer const&) = default;
  ImuSerializer& operator=(ImuSerializer const&) = default;
  ImuSerializer(ImuSerializer&&) = default;
  ImuSerializer& operator=(ImuSerializer&&) = default;

  int Serialize(const message::SerializeMsg* data_ptr, message::DeserializeMsg* msg_ptr) const override;
  int Deserialize(const message::DeserializeMsg* msg_ptr, message::SerializeMsg* data_ptr) const override;

private:
  static constexpr std::uint32_t kSecond2Ns{1000000000};
};

}

#endif