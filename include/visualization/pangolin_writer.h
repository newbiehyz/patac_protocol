/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/include/visualization/pangolin_writer.h
 * Path: /home/yukan/Documents/work/local_mapping/include/visualization
 * Created Date: Wednesday, May 14th 2025, 6:30:15 pm
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */
#pragma once
#include <pangolin/pangolin.h>

namespace name {
class PangolinWriter {
 public:
  PangolinWriter();
  static PangolinWriter &GetInstance();
  void Write();

 private:
};
}  // namespace name
