//

#include "interface/parking/cylinder_multitask_data.h"

#include <cstring>

namespace autodrive {

CylinderMultiTaskData::CylinderMultiTaskData() noexcept
    : CommonMsgData{}, cylinder_multitask_data_{} {}

CylinderMultiTaskData::CylinderMultiTaskData(std::int64_t timestamp) noexcept
    : CommonMsgData{timestamp}, cylinder_multitask_data_{} {}

CylinderMultiTaskData::CylinderMultiTask const&
CylinderMultiTaskData::GetCylinderMultiTask() const noexcept {
  return cylinder_multitask_data_;
}

CylinderMultiTaskData::CylinderMultiTask&
CylinderMultiTaskData::MutableCylinderMultiTask() noexcept {
  return cylinder_multitask_data_;
}

void CylinderMultiTaskData::SetCylinderMultiTask(
    CylinderMultiTask const& dynamic_obstacle_data) noexcept {
  cylinder_multitask_data_ = dynamic_obstacle_data;
}

void CylinderMultiTaskData::Clear() noexcept {
  SetFrameId(0L);
  SetTimestamp(0L);
  SetRawTimestamp(0L);
  memset(&cylinder_multitask_data_, 0, sizeof(cylinder_multitask_data_));
}

}  // namespace autodrive
