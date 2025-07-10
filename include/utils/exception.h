//
#ifndef AUTODRIVE_HYPERSAR_UTILS_EXCEPTION_H_
#define AUTODRIVE_HYPERSAR_UTILS_EXCEPTION_H_

#include <stdexcept>
#include <string>

namespace autodrive {

// Base exception class
class Exception : public std::exception {
 public:
  Exception() noexcept = default;
  explicit Exception(const char* what) noexcept;
  explicit Exception(std::string const& what) noexcept;
  explicit Exception(std::string&& what) noexcept;
  ~Exception() noexcept = default;
  Exception(Exception const&) noexcept;
  Exception& operator=(Exception const&) noexcept;
  Exception(Exception&&) noexcept;
  Exception& operator=(Exception&&) noexcept;

  const char* what() const noexcept override;

 private:
  /// @brief Exception message
  std::string msg_;
};

// Init exception
class InitException : public Exception {
 public:
  InitException() noexcept = default;
  explicit InitException(const char* what) noexcept;
  explicit InitException(std::string const& what) noexcept;
  explicit InitException(std::string&& what) noexcept;
  ~InitException() noexcept = default;
  InitException(InitException const&) noexcept = default;
  InitException& operator=(InitException const&) noexcept = default;
  InitException(InitException&&) noexcept = default;
  InitException& operator=(InitException&&) noexcept = default;
};

// IO exception
class IOException : public Exception {
 public:
  IOException() noexcept = default;
  explicit IOException(const char* what) noexcept;
  explicit IOException(std::string const& what) noexcept;
  explicit IOException(std::string&& what) noexcept;
  ~IOException() noexcept = default;
  IOException(IOException const&) noexcept = default;
  IOException& operator=(IOException const&) noexcept = default;
  IOException(IOException&&) noexcept = default;
  IOException& operator=(IOException&&) noexcept = default;
};

class RuntimeException : public Exception {
 public:
  RuntimeException() noexcept = default;
  explicit RuntimeException(const char* what) noexcept;
  explicit RuntimeException(std::string const& what) noexcept;
  explicit RuntimeException(std::string&& what) noexcept;
  ~RuntimeException() noexcept = default;
  RuntimeException(RuntimeException const&) noexcept = default;
  RuntimeException& operator=(RuntimeException const&) noexcept = default;
  RuntimeException(RuntimeException&&) noexcept = default;
  RuntimeException& operator=(RuntimeException&&) noexcept = default;
};

// Operation exception
class OperationException : public Exception {
 public:
  OperationException() noexcept = default;
  explicit OperationException(const char* what) noexcept;
  explicit OperationException(std::string const& what) noexcept;
  explicit OperationException(std::string&& what) noexcept;
  ~OperationException() noexcept = default;
  OperationException(OperationException const&) noexcept = default;
  OperationException& operator=(OperationException const&) noexcept = default;
  OperationException(OperationException&&) noexcept = default;
  OperationException& operator=(OperationException&&) noexcept = default;
};

// Resource exception
class ResourceException : public Exception {
 public:
  ResourceException() noexcept = default;
  explicit ResourceException(const char* what) noexcept;
  explicit ResourceException(std::string const& what) noexcept;
  explicit ResourceException(std::string&& what) noexcept;
  ~ResourceException() noexcept = default;
  ResourceException(ResourceException const&) noexcept = default;
  ResourceException& operator=(ResourceException const&) noexcept = default;
  ResourceException(ResourceException&&) noexcept = default;
  ResourceException& operator=(ResourceException&&) noexcept = default;
};

// Data exception
class DataException : public Exception {
 public:
  DataException() noexcept = default;
  explicit DataException(const char* what) noexcept;
  explicit DataException(std::string const& what) noexcept;
  explicit DataException(std::string&& what) noexcept;
  ~DataException() noexcept = default;
  DataException(DataException const&) noexcept = default;
  DataException& operator=(DataException const&) noexcept = default;
  DataException(DataException&&) noexcept = default;
  DataException& operator=(DataException&&) noexcept = default;
};

// Logic exception
class LogicException : public Exception {
 public:
  LogicException() noexcept = default;
  explicit LogicException(const char* what) noexcept;
  explicit LogicException(std::string const& what) noexcept;
  explicit LogicException(std::string&& what) noexcept;
  ~LogicException() noexcept = default;
  LogicException(LogicException const&) noexcept = default;
  LogicException& operator=(LogicException const&) noexcept = default;
  LogicException(LogicException&&) noexcept = default;
  LogicException& operator=(LogicException&&) noexcept = default;
};

// Invalid config
class InvalidConfigException : public InitException {
 public:
  InvalidConfigException() noexcept = default;
  explicit InvalidConfigException(const char* what) noexcept;
  explicit InvalidConfigException(std::string const& what) noexcept;
  explicit InvalidConfigException(std::string&& what) noexcept;
  ~InvalidConfigException() noexcept = default;
  InvalidConfigException(InvalidConfigException const&) noexcept = default;
  InvalidConfigException& operator=(InvalidConfigException const&)
      noexcept = default;
  InvalidConfigException(InvalidConfigException&&) noexcept = default;
  InvalidConfigException& operator=(InvalidConfigException&&) noexcept =
      default;
};

// Thread init failed
class ThreadInitFailedException : public InitException {
 public:
  ThreadInitFailedException() noexcept = default;
  explicit ThreadInitFailedException(const char* what) noexcept;
  explicit ThreadInitFailedException(std::string const& what) noexcept;
  explicit ThreadInitFailedException(std::string&& what) noexcept;
  ~ThreadInitFailedException() noexcept = default;
  ThreadInitFailedException(ThreadInitFailedException const&) noexcept =
      default;
  ThreadInitFailedException& operator=(ThreadInitFailedException const&)
      noexcept = default;
  ThreadInitFailedException(ThreadInitFailedException&&) noexcept = default;
  ThreadInitFailedException& operator=(ThreadInitFailedException&&) noexcept =
      default;
};

// Socket init failed
class SocketInitFailedException : public InitException {
 public:
  SocketInitFailedException() noexcept = default;
  explicit SocketInitFailedException(const char* what) noexcept;
  explicit SocketInitFailedException(std::string const& what) noexcept;
  explicit SocketInitFailedException(std::string&& what) noexcept;
  ~SocketInitFailedException() noexcept = default;
  SocketInitFailedException(SocketInitFailedException const&) noexcept =
      default;
  SocketInitFailedException& operator=(SocketInitFailedException const&)
      noexcept = default;
  SocketInitFailedException(SocketInitFailedException&&) noexcept = default;
  SocketInitFailedException& operator=(SocketInitFailedException&&) noexcept =
      default;
};

// Sensor init failed
class SensorInitFailedException : public InitException {
 public:
  SensorInitFailedException() noexcept = default;
  explicit SensorInitFailedException(const char* what) noexcept;
  explicit SensorInitFailedException(std::string const& what) noexcept;
  explicit SensorInitFailedException(std::string&& what) noexcept;
  ~SensorInitFailedException() noexcept = default;
  SensorInitFailedException(SensorInitFailedException const&) noexcept =
      default;
  SensorInitFailedException& operator=(SensorInitFailedException const&)
      noexcept = default;
  SensorInitFailedException(SensorInitFailedException&&) noexcept = default;
  SensorInitFailedException& operator=(SensorInitFailedException&&) noexcept =
      default;
};

// Redis init failed
class RedisInitFailedException : public InitException {
 public:
  RedisInitFailedException() noexcept = default;
  explicit RedisInitFailedException(const char* what) noexcept;
  explicit RedisInitFailedException(std::string const& what) noexcept;
  explicit RedisInitFailedException(std::string&& what) noexcept;
  ~RedisInitFailedException() noexcept = default;
  RedisInitFailedException(RedisInitFailedException const&) noexcept = default;
  RedisInitFailedException& operator=(RedisInitFailedException const&)
      noexcept = default;
  RedisInitFailedException(RedisInitFailedException&&) noexcept = default;
  RedisInitFailedException& operator=(RedisInitFailedException&&) noexcept =
      default;
};

// Mqtt init failed
class MqttInitFailedException : public InitException {
 public:
  MqttInitFailedException() noexcept = default;
  explicit MqttInitFailedException(const char* what) noexcept;
  explicit MqttInitFailedException(std::string const& what) noexcept;
  explicit MqttInitFailedException(std::string&& what) noexcept;
  ~MqttInitFailedException() noexcept = default;
  MqttInitFailedException(MqttInitFailedException const&) noexcept = default;
  MqttInitFailedException& operator=(MqttInitFailedException const&)
      noexcept = default;
  MqttInitFailedException(MqttInitFailedException&&) noexcept = default;
  MqttInitFailedException& operator=(MqttInitFailedException&&) noexcept =
      default;
};

// Aliyun Oss init failed
class AliyunOssInitFailedException : public InitException {
 public:
  AliyunOssInitFailedException() noexcept = default;
  explicit AliyunOssInitFailedException(const char* what) noexcept;
  explicit AliyunOssInitFailedException(std::string const& what) noexcept;
  explicit AliyunOssInitFailedException(std::string&& what) noexcept;
  ~AliyunOssInitFailedException() noexcept = default;
  AliyunOssInitFailedException(AliyunOssInitFailedException const&) noexcept =
      default;
  AliyunOssInitFailedException& operator=(AliyunOssInitFailedException const&)
      noexcept = default;
  AliyunOssInitFailedException(AliyunOssInitFailedException&&) noexcept = default;
  AliyunOssInitFailedException& operator=(AliyunOssInitFailedException&&)
      noexcept = default;
};

// Socket IO exception
class SocketIOException : public IOException {
 public:
  SocketIOException() noexcept = default;
  explicit SocketIOException(const char* what) noexcept;
  explicit SocketIOException(std::string const& what) noexcept;
  explicit SocketIOException(std::string&& what) noexcept;
  ~SocketIOException() noexcept = default;
  SocketIOException(SocketIOException const&) noexcept = default;
  SocketIOException& operator=(SocketIOException const&) noexcept = default;
  SocketIOException(SocketIOException&&) noexcept = default;
  SocketIOException& operator=(SocketIOException&&) noexcept = default;
};

// Can IO exception
class CanIOException : public IOException {
 public:
  CanIOException() noexcept = default;
  explicit CanIOException(const char* what) noexcept;
  explicit CanIOException(std::string const& what) noexcept;
  explicit CanIOException(std::string&& what) noexcept;
  ~CanIOException() noexcept = default;
  CanIOException(CanIOException const&) noexcept = default;
  CanIOException& operator=(CanIOException const&) noexcept = default;
  CanIOException(CanIOException&&) noexcept = default;
  CanIOException& operator=(CanIOException&&) noexcept = default;
};

// Serial IO exception
class SerialIOException : public IOException {
 public:
  SerialIOException() noexcept = default;
  explicit SerialIOException(const char* what) noexcept;
  explicit SerialIOException(std::string const& what) noexcept;
  explicit SerialIOException(std::string&& what) noexcept;
  ~SerialIOException() noexcept = default;
  SerialIOException(SerialIOException const&) noexcept = default;
  SerialIOException& operator=(SerialIOException const&) noexcept = default;
  SerialIOException(SerialIOException&&) noexcept = default;
  SerialIOException& operator=(SerialIOException&&) noexcept = default;
};

// Lvds IO exception
class LvdsIOException : public IOException {
 public:
  LvdsIOException() noexcept = default;
  explicit LvdsIOException(const char* what) noexcept;
  explicit LvdsIOException(std::string const& what) noexcept;
  explicit LvdsIOException(std::string&& what) noexcept;
  ~LvdsIOException() noexcept = default;
  LvdsIOException(LvdsIOException const&) noexcept = default;
  LvdsIOException& operator=(LvdsIOException const&) noexcept = default;
  LvdsIOException(LvdsIOException&&) noexcept = default;
  LvdsIOException& operator=(LvdsIOException&&) noexcept = default;
};

// Redis IO exception
class RedisIOException : public IOException {
 public:
  RedisIOException() noexcept = default;
  explicit RedisIOException(const char* what) noexcept;
  explicit RedisIOException(std::string const& what) noexcept;
  explicit RedisIOException(std::string&& what) noexcept;
  ~RedisIOException() noexcept = default;
  RedisIOException(RedisIOException const&) noexcept = default;
  RedisIOException& operator=(RedisIOException const&) noexcept = default;
  RedisIOException(RedisIOException&&) noexcept = default;
  RedisIOException& operator=(RedisIOException&&) noexcept = default;
};

// Pcie IO exception
class PcieIOException : public IOException {
 public:
  PcieIOException() noexcept = default;
  explicit PcieIOException(const char* what) noexcept;
  explicit PcieIOException(std::string const& what) noexcept;
  explicit PcieIOException(std::string&& what) noexcept;
  ~PcieIOException() noexcept = default;
  PcieIOException(PcieIOException const&) noexcept = default;
  PcieIOException& operator=(PcieIOException const&) noexcept = default;
  PcieIOException(PcieIOException&&) noexcept = default;
  PcieIOException& operator=(PcieIOException&&) noexcept = default;
};

// MQTT IO exception
class MqttIOException : public IOException {
 public:
  MqttIOException() noexcept = default;
  explicit MqttIOException(const char* what) noexcept;
  explicit MqttIOException(std::string const& what) noexcept;
  explicit MqttIOException(std::string&& what) noexcept;
  ~MqttIOException() noexcept = default;
  MqttIOException(MqttIOException const&) noexcept = default;
  MqttIOException& operator=(MqttIOException const&) noexcept = default;
  MqttIOException(MqttIOException&&) noexcept = default;
  MqttIOException& operator=(MqttIOException&&) noexcept = default;
};

// Azure IoT IO exception
class AliyunOssIOException : public IOException {
 public:
  AliyunOssIOException() noexcept = default;
  explicit AliyunOssIOException(const char* what) noexcept;
  explicit AliyunOssIOException(std::string const& what) noexcept;
  explicit AliyunOssIOException(std::string&& what) noexcept;
  ~AliyunOssIOException() noexcept = default;
  AliyunOssIOException(AliyunOssIOException const&) noexcept = default;
  AliyunOssIOException& operator=(AliyunOssIOException const&) noexcept = default;
  AliyunOssIOException(AliyunOssIOException&&) noexcept = default;
  AliyunOssIOException& operator=(AliyunOssIOException&&) noexcept = default;
};

// Channel not opened
class ChannelNotOpenedException : public OperationException {
 public:
  ChannelNotOpenedException() noexcept = default;
  explicit ChannelNotOpenedException(const char* what) noexcept;
  explicit ChannelNotOpenedException(std::string const& what) noexcept;
  explicit ChannelNotOpenedException(std::string&& what) noexcept;
  ~ChannelNotOpenedException() noexcept = default;
  ChannelNotOpenedException(ChannelNotOpenedException const&) noexcept =
      default;
  ChannelNotOpenedException& operator=(ChannelNotOpenedException const&)
      noexcept = default;
  ChannelNotOpenedException(ChannelNotOpenedException&&) noexcept = default;
  ChannelNotOpenedException& operator=(ChannelNotOpenedException&&) noexcept =
      default;
};

// Invalid channel operation
class InvalidChannelOperationException : public OperationException {
 public:
  InvalidChannelOperationException() noexcept = default;
  explicit InvalidChannelOperationException(const char* what) noexcept;
  explicit InvalidChannelOperationException(std::string const& what) noexcept;
  explicit InvalidChannelOperationException(std::string&& what) noexcept;
  ~InvalidChannelOperationException() noexcept = default;
  InvalidChannelOperationException(InvalidChannelOperationException const&)
      noexcept = default;
  InvalidChannelOperationException& operator=(
      InvalidChannelOperationException const&) noexcept = default;
  InvalidChannelOperationException(InvalidChannelOperationException&&)
      noexcept = default;
  InvalidChannelOperationException& operator=(
      InvalidChannelOperationException&&) noexcept = default;
};

// Invalid resource
class InvalidResourceException : public ResourceException {
 public:
  InvalidResourceException() noexcept = default;
  explicit InvalidResourceException(const char* what) noexcept;
  explicit InvalidResourceException(std::string const& what) noexcept;
  explicit InvalidResourceException(std::string&& what) noexcept;
  ~InvalidResourceException() noexcept = default;
  InvalidResourceException(InvalidResourceException const&) noexcept = default;
  InvalidResourceException& operator=(InvalidResourceException const&)
      noexcept = default;
  InvalidResourceException(InvalidResourceException&&) noexcept = default;
  InvalidResourceException& operator=(InvalidResourceException&&) noexcept =
      default;
};

// Resource not exist
class ResourceNotExistException : public ResourceException {
 public:
  ResourceNotExistException() noexcept = default;
  explicit ResourceNotExistException(const char* what) noexcept;
  explicit ResourceNotExistException(std::string const& what) noexcept;
  explicit ResourceNotExistException(std::string&& what) noexcept;
  ~ResourceNotExistException() noexcept = default;
  ResourceNotExistException(ResourceNotExistException const&) noexcept = default;
  ResourceNotExistException& operator=(ResourceNotExistException const&)
      noexcept = default;
  ResourceNotExistException(ResourceNotExistException&&) noexcept = default;
  ResourceNotExistException& operator=(ResourceNotExistException&&) noexcept =
      default;
};

// Resource already exist
class ResourceAlreadyExistException : public ResourceException {
 public:
  ResourceAlreadyExistException() noexcept = default;
  explicit ResourceAlreadyExistException(const char* what) noexcept;
  explicit ResourceAlreadyExistException(std::string const& what) noexcept;
  explicit ResourceAlreadyExistException(std::string&& what) noexcept;
  ~ResourceAlreadyExistException() noexcept = default;
  ResourceAlreadyExistException(ResourceAlreadyExistException const&)
      noexcept = default;
  ResourceAlreadyExistException& operator=(ResourceAlreadyExistException const&)
      noexcept = default;
  ResourceAlreadyExistException(ResourceAlreadyExistException&&) noexcept =
      default;
  ResourceAlreadyExistException& operator=(ResourceAlreadyExistException&&)
      noexcept = default;
};

// Out of resource
class OutOfResourceException : public ResourceException {
 public:
  OutOfResourceException() noexcept = default;
  explicit OutOfResourceException(const char* what) noexcept;
  explicit OutOfResourceException(std::string const& what) noexcept;
  explicit OutOfResourceException(std::string&& what) noexcept;
  ~OutOfResourceException() noexcept = default;
  OutOfResourceException(OutOfResourceException const&) noexcept = default;
  OutOfResourceException& operator=(OutOfResourceException const&) noexcept =
      default;
  OutOfResourceException(OutOfResourceException&&) noexcept = default;
  OutOfResourceException& operator=(OutOfResourceException&&) noexcept =
      default;
};

// File not found
class FileNotFoundException : public ResourceException {
 public:
  FileNotFoundException() noexcept = default;
  explicit FileNotFoundException(const char* what) noexcept;
  explicit FileNotFoundException(std::string const& what) noexcept;
  explicit FileNotFoundException(std::string&& what) noexcept;
  ~FileNotFoundException() noexcept = default;
  FileNotFoundException(FileNotFoundException const&) noexcept = default;
  FileNotFoundException& operator=(FileNotFoundException const&) noexcept =
      default;
  FileNotFoundException(FileNotFoundException&&) noexcept = default;
  FileNotFoundException& operator=(FileNotFoundException&&) noexcept = default;
};

// Invalid calibration
class InvalidCalibrationException : public DataException {
 public:
  InvalidCalibrationException() noexcept = default;
  explicit InvalidCalibrationException(const char* what) noexcept;
  explicit InvalidCalibrationException(std::string const& what) noexcept;
  explicit InvalidCalibrationException(std::string&& what) noexcept;
  ~InvalidCalibrationException() noexcept = default;
  InvalidCalibrationException(InvalidCalibrationException const&) noexcept =
      default;
  InvalidCalibrationException& operator=(InvalidCalibrationException const&)
      noexcept = default;
  InvalidCalibrationException(InvalidCalibrationException&&) noexcept =
      default;
  InvalidCalibrationException& operator=(InvalidCalibrationException&&)
      noexcept = default;
};

// Invalid argument
class InvalidArgumentException : public DataException {
 public:
  InvalidArgumentException() noexcept = default;
  explicit InvalidArgumentException(const char* what) noexcept;
  explicit InvalidArgumentException(std::string const& what) noexcept;
  explicit InvalidArgumentException(std::string&& what) noexcept;
  ~InvalidArgumentException() noexcept = default;
  InvalidArgumentException(InvalidArgumentException const&) noexcept = default;
  InvalidArgumentException& operator=(InvalidArgumentException const&)
      noexcept = default;
  InvalidArgumentException(InvalidArgumentException&&) noexcept = default;
  InvalidArgumentException& operator=(InvalidArgumentException&&) noexcept =
      default;
};

// Invalid data
class InvalidDataException : public DataException {
 public:
  InvalidDataException() noexcept = default;
  explicit InvalidDataException(const char* what) noexcept;
  explicit InvalidDataException(std::string const& what) noexcept;
  explicit InvalidDataException(std::string&& what) noexcept;
  ~InvalidDataException() noexcept = default;
  InvalidDataException(InvalidDataException const&) noexcept = default;
  InvalidDataException& operator=(InvalidDataException const&) noexcept =
      default;
  InvalidDataException(InvalidDataException&&) noexcept = default;
  InvalidDataException& operator=(InvalidDataException&&) noexcept = default;
};

}

#endif // AUTODRIVE_HYPERSAR_UTILS_EXCEPTION_H_

/* vim: set ts=2 sw=2 sts=2 tw=80 et: */
