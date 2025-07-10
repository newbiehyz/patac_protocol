//
#ifndef AUTODRIVE_MESSAGE_SERALIZER_H
#define AUTODRIVE_MESSAGE_SERALIZER_H

#include <memory>
#include "message.h"

namespace autodrive {
namespace message {

template <typename DataType>
class SerializeDataMsg : public Message {
 public:
  SerializeDataMsg(DataType* data_ptr) noexcept : data_ptr{data_ptr} {}
  DataType* GetData() const noexcept;
  DataType* MutableData() noexcept;

 private:
  DataType* data_ptr;
};

template <typename DataType>
DataType* SerializeDataMsg<DataType>::GetData() const noexcept {
  return data_ptr;
}

template <typename DataType>
DataType* SerializeDataMsg<DataType>::MutableData() noexcept {
  return data_ptr;
}

template <typename MsgType>
class DeserializeDataMsg : public Message {
 public:
  DeserializeDataMsg(MsgType* msg_ptr) noexcept : msg_ptr{msg_ptr} {}
  MsgType* GetMsg() const noexcept;
  MsgType* MutableMsg() noexcept;

 private:
  MsgType* msg_ptr;
};

template <typename MsgType>
MsgType* DeserializeDataMsg<MsgType>::GetMsg() const noexcept {
  return msg_ptr;
}

template <typename MsgType>
MsgType* DeserializeDataMsg<MsgType>::MutableMsg() noexcept {
  return msg_ptr;
}

using DeserializeMsg = Message;
using SerializeMsg = Message;

class SerializerPolicy {
public:
  virtual ~SerializerPolicy() = default;
  SerializerPolicy() = default;
  SerializerPolicy(const SerializerPolicy &) = delete;
  SerializerPolicy &operator=(const SerializerPolicy &) = delete;
  SerializerPolicy(SerializerPolicy &&) = delete;
  SerializerPolicy &operator=(SerializerPolicy &&) = delete;

  virtual int Serialize(const SerializeMsg* data_ptr, DeserializeMsg* msg_ptr) const = 0;
  virtual int Deserialize(const DeserializeMsg* msg_ptr, SerializeMsg* data_ptr) const = 0;
};

class DefaultSerializer : public SerializerPolicy {
public:
  int Serialize(const SerializeMsg* data_ptr, DeserializeMsg* msg_ptr) const override;
  int Deserialize(const DeserializeMsg* msg_ptr, SerializeMsg* data_ptr) const override;
};

}
}

#endif