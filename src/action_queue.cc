/*
 * Filename: /home/yukan/Documents/work/local_mapping/src/action_queue.cc
 * Path: /home/yukan/Documents/work/local_mapping/src
 * Created Date: Wednesday, June 11th 2025, 3:43:03 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "action_queue.h"

namespace apa_slam {
ActionQueue::ActionQueue() {}

void ActionQueue::PushAction(const Action& ac) {
  std::lock_guard<std::mutex> lock(_data_mutex);
  _action_queue.push_back(ac);

  if (_action_queue.size() > QUEUE_MAX_LEN) {
    _action_queue.pop_front();
  }
}

bool ActionQueue::GetNextAction(Action& ac) {
  std::lock_guard<std::mutex> lock(_data_mutex);

  if (_action_queue.empty()) {
    return false;
  }
  ac = _action_queue.front();
  _action_queue.pop_front();
  return true;
}
void ActionQueue::Clear() {
  std::lock_guard<std::mutex> lock(_data_mutex);
  _action_queue.clear();
}

ActionQueue& ActionQueue::GetInstance() {
  static ActionQueue instance;
  return instance;
}
}  // namespace apa_slam
