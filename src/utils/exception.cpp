//
#include "utils/exception.h"

#include <stdexcept>
#include <string>

namespace autodrive {

Exception::Exception(const char* what) noexcept : std::exception{},
    msg_{what} {}

Exception::Exception(std::string const& what) noexcept :
    std::exception{}, msg_{what} {}

Exception::Exception(std::string&& what) noexcept :
    std::exception{}, msg_{what.c_str()} {}

Exception::Exception(Exception const& exception) noexcept {
  msg_ = exception.msg_;
}

Exception& Exception::operator=(Exception const& exception) noexcept {
  if (this == &exception)
    return *this;
  msg_ = exception.msg_;
  return *this;
}

Exception::Exception(Exception&& exception) noexcept {
  msg_ = exception.msg_;
}

Exception& Exception::operator=(Exception&& exception) noexcept {
  if (this == &exception)
    return *this;
  msg_ = exception.msg_;
  return *this;
}

const char* Exception::what() const noexcept {
  return msg_.c_str();
}

InitException::InitException(const char* what) noexcept :
    Exception{what} {}

InitException::InitException(std::string const& what) noexcept :
    Exception{what} {}

InitException::InitException(std::string&& what) noexcept :
    Exception{what} {}

IOException::IOException(const char* what) noexcept :
    Exception{what} {}

IOException::IOException(std::string const& what) noexcept :
    Exception{what} {}

IOException::IOException(std::string&& what) noexcept :
    Exception{what} {}

RuntimeException::RuntimeException(const char* what) noexcept :
    Exception{what} {}

RuntimeException::RuntimeException(std::string const& what) noexcept :
    Exception{what} {}

RuntimeException::RuntimeException(std::string&& what) noexcept :
    Exception{what} {}

OperationException::OperationException(const char* what) noexcept :
    Exception{what} {}

OperationException::OperationException(std::string const& what) noexcept :
    Exception{what} {}

ResourceException::ResourceException(const char* what) noexcept :
    Exception{what} {}

ResourceException::ResourceException(std::string const& what) noexcept :
    Exception{what} {}

ResourceException::ResourceException(std::string&& what) noexcept :
    Exception{what} {}

DataException::DataException(const char* what) noexcept :
    Exception{what} {}

DataException::DataException(std::string const& what)
    noexcept : Exception{what} {}

DataException::DataException(std::string&& what)
    noexcept : Exception{what} {}

LogicException::LogicException(const char* what) noexcept :
    Exception{what} {}

LogicException::LogicException(std::string const& what) noexcept :
    Exception{what} {}

LogicException::LogicException(std::string&& what) noexcept :
    Exception{what} {}

InvalidConfigException::InvalidConfigException(const char* what) noexcept :
    InitException{what} {}

InvalidConfigException::InvalidConfigException(std::string const& what)
    noexcept : InitException{what} {}

InvalidConfigException::InvalidConfigException(std::string&& what)
    noexcept : InitException{what} {}

ThreadInitFailedException::ThreadInitFailedException(const char* what)
    noexcept : InitException{what} {}

ThreadInitFailedException::ThreadInitFailedException(std::string const& what)
    noexcept : InitException{what} {}

ThreadInitFailedException::ThreadInitFailedException(std::string&& what)
    noexcept : InitException{what} {}

SocketInitFailedException::SocketInitFailedException(const char* what)
    noexcept : InitException{what} {}

SocketInitFailedException::SocketInitFailedException(std::string const& what)
    noexcept : InitException{what} {}

SocketInitFailedException::SocketInitFailedException(std::string&& what)
    noexcept : InitException{what} {}

SensorInitFailedException::SensorInitFailedException(const char* what)
    noexcept : InitException{what} {}

SensorInitFailedException::SensorInitFailedException(std::string const& what)
    noexcept : InitException{what} {}

SensorInitFailedException::SensorInitFailedException(std::string&& what)
    noexcept : InitException{what} {}

RedisInitFailedException::RedisInitFailedException(const char* what)
    noexcept : InitException{what} {}

RedisInitFailedException::RedisInitFailedException(std::string const& what)
    noexcept : InitException{what} {}

RedisInitFailedException::RedisInitFailedException(std::string&& what)
    noexcept : InitException{what} {}

MqttInitFailedException::MqttInitFailedException(const char* what)
    noexcept : InitException{what} {}

MqttInitFailedException::MqttInitFailedException(std::string const& what)
    noexcept : InitException{what} {}

MqttInitFailedException::MqttInitFailedException(std::string&& what)
    noexcept : InitException{what} {}

AliyunOssInitFailedException::AliyunOssInitFailedException(const char* what)
    noexcept : InitException{what} {}

AliyunOssInitFailedException::AliyunOssInitFailedException(
    std::string const& what) noexcept : InitException{what} {}

AliyunOssInitFailedException::AliyunOssInitFailedException(
    std::string&& what) noexcept : InitException{what} {}

SocketIOException::SocketIOException(const char* what) noexcept :
    IOException{what} {}

SocketIOException::SocketIOException(std::string const& what) noexcept :
    IOException{what} {}

SocketIOException::SocketIOException(std::string&& what) noexcept :
    IOException{what} {}

CanIOException::CanIOException(const char* what) noexcept :
    IOException{what} {}

CanIOException::CanIOException(std::string const& what) noexcept :
    IOException{what} {}

CanIOException::CanIOException(std::string&& what) noexcept :
    IOException{what} {}

SerialIOException::SerialIOException(const char* what) noexcept :
    IOException{what} {}

SerialIOException::SerialIOException(std::string const& what) noexcept :
    IOException{what} {}

SerialIOException::SerialIOException(std::string&& what) noexcept :
    IOException{what} {}

LvdsIOException::LvdsIOException(const char* what) noexcept :
    IOException{what} {}

LvdsIOException::LvdsIOException(std::string const& what) noexcept :
    IOException{what} {}

LvdsIOException::LvdsIOException(std::string&& what) noexcept :
    IOException{what} {}

RedisIOException::RedisIOException(const char* what) noexcept :
    IOException{what} {}

RedisIOException::RedisIOException(std::string const& what) noexcept :
    IOException{what} {}

RedisIOException::RedisIOException(std::string&& what) noexcept :
    IOException{what} {}

PcieIOException::PcieIOException(const char* what) noexcept :
    IOException{what} {}

PcieIOException::PcieIOException(std::string const& what) noexcept :
    IOException{what} {}

PcieIOException::PcieIOException(std::string&& what) noexcept :
    IOException{what} {}

MqttIOException::MqttIOException(const char* what) noexcept :
    IOException{what} {}

MqttIOException::MqttIOException(std::string const& what) noexcept :
    IOException{what} {}

MqttIOException::MqttIOException(std::string&& what) noexcept :
    IOException{what} {}

AliyunOssIOException::AliyunOssIOException(const char* what)
    noexcept : IOException{what} {}

AliyunOssIOException::AliyunOssIOException(
    std::string const& what) noexcept : IOException{what} {}

AliyunOssIOException::AliyunOssIOException(
    std::string&& what) noexcept : IOException{what} {}

ChannelNotOpenedException::ChannelNotOpenedException(const char* what)
    noexcept : OperationException{what} {}

ChannelNotOpenedException::ChannelNotOpenedException(std::string const& what)
    noexcept : OperationException{what} {}

ChannelNotOpenedException::ChannelNotOpenedException(std::string&& what)
    noexcept : OperationException{what} {}

InvalidChannelOperationException::InvalidChannelOperationException(
    const char* what) noexcept : OperationException{what} {}

InvalidChannelOperationException::InvalidChannelOperationException(
    std::string const& what) noexcept : OperationException{what} {}

InvalidChannelOperationException::InvalidChannelOperationException(
    std::string&& what) noexcept : OperationException{what} {}

InvalidResourceException::InvalidResourceException(const char* what)
    noexcept : ResourceException{what} {}

InvalidResourceException::InvalidResourceException(std::string const& what)
    noexcept : ResourceException{what} {}

InvalidResourceException::InvalidResourceException(std::string&& what)
    noexcept : ResourceException{what} {}

ResourceNotExistException::ResourceNotExistException(const char* what)
    noexcept : ResourceException{what} {}

ResourceNotExistException::ResourceNotExistException(std::string const& what)
    noexcept : ResourceException{what} {}

ResourceNotExistException::ResourceNotExistException(std::string&& what)
    noexcept : ResourceException{what} {}

ResourceAlreadyExistException::ResourceAlreadyExistException(const char* what)
    noexcept : ResourceException{what} {}

ResourceAlreadyExistException::ResourceAlreadyExistException(
    std::string const& what) noexcept : ResourceException{what} {}

ResourceAlreadyExistException::ResourceAlreadyExistException(
    std::string&& what) noexcept : ResourceException{what} {}

OutOfResourceException::OutOfResourceException(const char* what) noexcept :
    ResourceException{what} {}

OutOfResourceException::OutOfResourceException(std::string const& what)
    noexcept : ResourceException{what} {}

OutOfResourceException::OutOfResourceException(std::string&& what)
    noexcept : ResourceException{what} {}

FileNotFoundException::FileNotFoundException(const char* what) noexcept :
    ResourceException{what} {}

FileNotFoundException::FileNotFoundException(std::string const& what)
    noexcept : ResourceException{what} {}

FileNotFoundException::FileNotFoundException(std::string&& what)
    noexcept : ResourceException{what} {}

InvalidCalibrationException::InvalidCalibrationException(const char* what)
    noexcept : DataException{what} {}

InvalidCalibrationException::InvalidCalibrationException(
    std::string const& what) noexcept : DataException{what} {}

InvalidCalibrationException::InvalidCalibrationException(
    std::string&& what) noexcept : DataException{what} {}

InvalidArgumentException::InvalidArgumentException(const char* what) noexcept :
    DataException{what} {}

InvalidArgumentException::InvalidArgumentException(std::string const& what)
    noexcept : DataException{what} {}

InvalidArgumentException::InvalidArgumentException(std::string&& what)
    noexcept : DataException{what} {}

InvalidDataException::InvalidDataException(const char* what) noexcept :
    DataException{what} {}

InvalidDataException::InvalidDataException(std::string const& what) noexcept :
    DataException{what} {}

InvalidDataException::InvalidDataException(std::string&& what) noexcept :
    DataException{what} {}

}

/* vim: set ts=2 sw=2 sts=2 tw=100 et: */
