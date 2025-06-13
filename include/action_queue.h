/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/action_queue.h
 * Path: /home/yukan/Documents/work/local_mapping/include
 * Created Date: Wednesday, June 11th 2025, 3:30:55 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once
#include <mutex>
#include <deque>

#include "local_mapping_define.h"

#define QUEUE_MAX_LEN 5

namespace apa_slam {
class ActionQueue {
 public:
  ActionQueue();
  void PushAction(const Action& ac);
  bool GetNextAction(Action& ac);
  void Clear();
  static ActionQueue& GetInstance();

 private:
  std::deque<Action> _action_queue;
  std::mutex _data_mutex;
};
}  // namespace apa_slam
