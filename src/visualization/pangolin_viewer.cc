/*
 * Filename:
 * /home/yukan/Documents/work/local_mapping/src/visualization/pangolin_viewer.cc
 * Path: /home/yukan/Documents/work/local_mapping/src/visualization
 * Created Date: Friday, May 9th 2025, 11:10:42 am
 * Author: yukan
 *
 * Copyright (c) 2025 PATAC
 */

#include "visualization/pangolin_viewer.h"

namespace apa_slam {
PangolinViewer::PangolinViewer() {
  _drawer = std::make_shared<PangolinDrawer>();
}

void PangolinViewer::Start() {
  std::thread th(&PangolinViewer::run, this);
  th.detach();
}

void PangolinViewer::run() {
  pangolin::CreateWindowAndBind("APA Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    _drawer->DrawAPA();

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    pangolin::FinishFrame();
  }
}
}  // namespace apa_slam
