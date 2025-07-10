//
#ifndef AUTODRIVE_MESSAGE_H
#define AUTODRIVE_MESSAGE_H

#include <memory>

namespace autodrive {
namespace message {

class DataRef;

using DataRefSp = std::shared_ptr<DataRef>;

class Message {
 public:
  Message() noexcept;

  Message(const Message &) = default;
  Message &operator=(const Message &) = default;
  Message(Message &&) = default;
  Message &operator=(Message &&) = default;

  virtual ~Message() = default;

  void SetData(DataRefSp const& data) noexcept;
  DataRefSp GetData() const noexcept;

 protected:
  DataRefSp data_;
};

}
}

#endif