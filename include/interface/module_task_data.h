//
#ifndef AUTODRIVE_INTERFACE_MODULE_TASK_DATA_H_
#define AUTODRIVE_INTERFACE_MODULE_TASK_DATA_H_

#include <foundation/type/common_msg_data.h>

namespace autodrive {

class ModuleTaskData final : public CommonMsgData {
 public:
  /// @brief Constructor
  ///
  /// @param timestamp  data time stamp
  explicit ModuleTaskData(std::int64_t timestamp) noexcept;

  ModuleTaskData() noexcept;
  ~ModuleTaskData() noexcept final = default;
  ModuleTaskData(ModuleTaskData const&) = default;
  ModuleTaskData& operator=(ModuleTaskData const&) = default;
  ModuleTaskData(ModuleTaskData&&) = default;
  ModuleTaskData& operator=(ModuleTaskData&&) = default;

  /// @brief Set dag id
  ///
  /// @param dag_id
  void SetDagId(std::uint32_t dag_id) noexcept;

  /// @brief Get dag id
  ///
  /// @return dag id
  std::uint32_t const& GetDagId() noexcept;

  /// @brief Set module id
  ///
  /// @param module_id
  void SetModuleId(std::uint32_t module_id) noexcept;

  /// @brief Get module id
  ///
  /// @return module id
  std::uint32_t const& GetModuleId() noexcept;

  /// @brief Set loop start time
  ///
  /// @param start_time
  void SetStartTime(std::uint64_t start_time) noexcept;

  /// @brief Get loop start time
  ///
  /// @return loop start time
  std::uint64_t const& GetStartTime() noexcept;

  /// @brief Set loop end time
  ///
  /// @param end_time
  void SetEndTime(std::uint64_t end_time) noexcept;

  /// @brief Get loop end time
  ///
  /// @return loop end time
  std::uint64_t const& GetEndTime() noexcept;

  /// @brief Clear data
  void Clear() noexcept;

 private:
  /// @brief DAG id
  std::uint32_t dag_id_;
  /// @brief module id
  std::uint32_t module_id_;
  /// @brief loop start time
  std::uint64_t start_time_;
  /// @brief loop end time
  std::uint64_t end_time_;
};
}

#endif // AUTODRIVE_INTERFACE_MODULE_TASK_DATA_H_