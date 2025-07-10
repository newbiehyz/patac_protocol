//
#ifndef AUTODRIVE_MESSAGE_FLEX_RAY_H
#define AUTODRIVE_MESSAGE_FLEX_RAY_H

#include <memory>
#include "message.h"

namespace autodrive {
namespace message {

template <typename DataType>
class FlexRayDataMsg : public Message {
 public:
  FlexRayDataMsg(DataType* data_ptr) noexcept : data_ptr{data_ptr} {}
  DataType* GetData() const noexcept;
  DataType* MutableData() noexcept;

 private:
  DataType* data_ptr;
};

template <typename DataType>
DataType* FlexRayDataMsg<DataType>::GetData() const noexcept {
  return data_ptr;
}

template <typename DataType>
DataType* FlexRayDataMsg<DataType>::MutableData() noexcept {
  return data_ptr;
}

template <typename MsgType>
class RawFlexRayDataMsg : public Message {
 public:
  RawFlexRayDataMsg(MsgType* msg_ptr) noexcept : msg_ptr{msg_ptr} {}
  MsgType* GetMsg() const noexcept;
  MsgType* MutableMsg() noexcept;

 private:
  MsgType* msg_ptr;
};

template <typename MsgType>
MsgType* RawFlexRayDataMsg<MsgType>::GetMsg() const noexcept {
  return msg_ptr;
}

template <typename MsgType>
MsgType* RawFlexRayDataMsg<MsgType>::MutableMsg() noexcept {
  return msg_ptr;
}

using ProvideMsg = Message;
using MappingMsg = Message;

class FlexRayPolicy {
 public:
  virtual ~FlexRayPolicy() = default;
  FlexRayPolicy() = default;
  FlexRayPolicy(const FlexRayPolicy &) = delete;
  FlexRayPolicy &operator=(const FlexRayPolicy &) = delete;
  FlexRayPolicy(FlexRayPolicy &&) = delete;
  FlexRayPolicy &operator=(FlexRayPolicy &&) = delete;

  virtual int Mapping(const ProvideMsg* msg_ptr, MappingMsg* data_ptr) const = 0;
};

class DefaultFlexRay : public FlexRayPolicy {
 public:
  int Mapping(const ProvideMsg* msg_ptr, MappingMsg* data_ptr) const override;
};

}
}

#endif