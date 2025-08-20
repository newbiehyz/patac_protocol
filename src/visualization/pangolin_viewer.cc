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
#include "map_io.h"
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

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, 0.2);

  pangolin::Var<bool> save_map_button("ui.SaveMap", false, false);
  pangolin::Var<bool> start_mapping_button("ui.StartMapping", false, false);
  pangolin::Var<bool> start_localization_button("ui.StartLocalization", false, false);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    if (pangolin::Pushed(save_map_button)) {
      {
        std::lock_guard<std::mutex> lock(vis_meas.meas_mutex);
        vis_meas.saveMap = true; // not used
        ActionQueue::GetInstance().PushAction(SAVE); // SAVE action reserved
        MapIO::GetInstance().SaveMapData();
        
        save_map_button.Reset();
        ActionQueue::GetInstance().PushAction(RESET);
      }
    }
    if (pangolin::Pushed(start_mapping_button)) {
      std::lock_guard<std::mutex> lock(vis_meas.meas_mutex);
      vis_meas.startMapping = true;
      only_localization = false;
      vis_meas.startLocalization = false;
      vis_meas.IsLoadMap = false;
      start_mapping_button.Reset();
    }
    if (pangolin::Pushed(start_localization_button)) {
      std::lock_guard<std::mutex> lock(vis_meas.meas_mutex);
      vis_meas.startLocalization = true;
      only_localization = true;
      vis_meas.startMapping = false;
      MapIO::GetInstance().LoadMapData(vis_meas.slot_map_data_filename, "");
      vis_meas.IsLoadMap = true;
      start_localization_button.Reset();
    }

    _drawer->DrawAPA(s_cam);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    pangolin::FinishFrame();
  }
}
} // namespace apa_slam
