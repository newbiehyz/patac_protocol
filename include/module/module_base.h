//

#ifndef AUTODRIVE_MODULE_BASE_
#define AUTODRIVE_MODULE_BASE_

#include <functional>
#include <memory>

namespace autodrive {

class ModuleBase {
 public:
  enum class ModuleType : std::uint8_t {
    NON_ACTIVE_MODULE = 0,
    TIMED_MODULE = 1,
    TRIGGER_MODULE = 2
  };

  struct OutputSignal {
    std::uint8_t module_id;
    std::uint8_t slot_id;
  };

  struct SignalCallbackFunc {
    void operator()(OutputSignal const& signal, std::uint8_t status = UINT8_MAX) {
      f(signal, status);
    };
    std::function<void(OutputSignal const&, std::uint8_t status)> f;
  };

  using SignalCallback = SignalCallbackFunc;
  using IdType = std::uint8_t;

  virtual ~ModuleBase() noexcept = default;
  ModuleBase(ModuleBase const&) = delete;
  ModuleBase& operator=(ModuleBase const&) = delete;
  ModuleBase(ModuleBase&&) = delete;
  ModuleBase& operator=(ModuleBase&&) = delete;

  /// @brief init module
  virtual void Init() noexcept(false) = 0;

  /// @brief destroy
  virtual void Destroy() noexcept = 0;

  /// @brief spin once
  virtual void SpinOnce() noexcept(false) = 0;

  /// @brief regist output signal callback
  /// 
  /// @param SignalCallback signal callback
  virtual void RegistOutputSignalCallback(SignalCallback callback) noexcept = 0;

  /// @brief get module id
  ///
  /// @return module id
  virtual IdType GetModuleId() noexcept = 0;

  /// @brief get module type
  ///
  /// @return module type
  virtual ModuleBase::ModuleType GetModuleType() noexcept = 0;

 protected:
  ModuleBase() = default;

};    


}

#endif // AUTODRIVE_MODULE_BASE_
