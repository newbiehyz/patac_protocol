/*
 * Filename: /home/yukan/Documents/work/local_mapping/include/visualization/pangolin_viewer.h
 * Path: /home/yukan/Documents/work/local_mapping/include/visualization
 * Created Date: Friday, May 9th 2025, 11:10:30 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#pragma once

#include <memory>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

#include "pangolin_drawer.h"

namespace apa_slam
{
  class PangolinViewer {
  public:
    typedef std::shared_ptr<PangolinViewer> Ptr;
    PangolinViewer();
    void Start();
  private:
    void run();
    PangolinDrawer::Ptr _drawer;
  };
} // namespace apa_slam
