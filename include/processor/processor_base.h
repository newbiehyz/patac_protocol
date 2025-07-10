//

#ifndef AUTODRIVE_DATA_FLOW_PROCESSOR_BASE_
#define AUTODRIVE_DATA_FLOW_PROCESSOR_BASE_

#include <functional>
#include <string>

namespace autodrive {

class ProcessorInfoData;

class ProcessorBase {
 public:
  using TimerNowFunc = std::function<std::uint64_t()>;
  using ProcessorInfoCallback = std::function<void(ProcessorInfoData const&)>;

  virtual ~ProcessorBase() noexcept = default;
  ProcessorBase(ProcessorBase const&) = delete;
  ProcessorBase& operator=(ProcessorBase const&) = delete;
  ProcessorBase(ProcessorBase&&) = delete;
  ProcessorBase& operator=(ProcessorBase&&) = delete;

  /// @brief init
  virtual void Init(std::string const& config_file) noexcept(false) = 0;

  /// @brief destroy
  virtual void Destroy() noexcept = 0;

  /// @brief spin
  virtual void RegistTimerNow(TimerNowFunc now) noexcept = 0;

  virtual void RegistProcessorInfoCallback(ProcessorInfoCallback callback) noexcept = 0;

 protected:
  ProcessorBase() = default;
};

}
#endif // AUTODRIVE_DATA_FLOW_PROCESSOR_BASE_
