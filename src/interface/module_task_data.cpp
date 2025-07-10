//

#include "interface/module_task_data.h"

namespace autodrive {

ModuleTaskData::ModuleTaskData() noexcept : CommonMsgData{}, 
    dag_id_{0U}, module_id_{0U}, start_time_{0U}, end_time_{0U} {}

ModuleTaskData::ModuleTaskData(std::int64_t timestamp) noexcept : CommonMsgData{timestamp}, 
    dag_id_{0U}, module_id_{0U}, start_time_{0U}, end_time_{0U} {}

void ModuleTaskData::SetDagId(std::uint32_t dag_id) noexcept {
  dag_id_ = dag_id;
}

std::uint32_t const& ModuleTaskData::GetDagId() noexcept {
  return dag_id_;
}

void ModuleTaskData::SetModuleId(std::uint32_t module_id) noexcept {
  module_id_ = module_id;    
}

std::uint32_t const& ModuleTaskData::GetModuleId() noexcept {
  return module_id_;  
}

void ModuleTaskData::SetStartTime(std::uint64_t start_time) noexcept {
  start_time_ = start_time;  
}

std::uint64_t const& ModuleTaskData::GetStartTime() noexcept {
  return start_time_;  
}

void ModuleTaskData::SetEndTime(std::uint64_t end_time) noexcept {
  end_time_ = end_time;  
}

std::uint64_t const& ModuleTaskData::GetEndTime() noexcept {
  return end_time_;   
}

void ModuleTaskData::Clear() noexcept {
  module_id_ = 0U;
  start_time_ = 0U;
  end_time_ = 0U;  
}

}