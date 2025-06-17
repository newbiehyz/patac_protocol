#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include "local_mapping_define.h"
#include <pangolin/pangolin.h>
using namespace apa_slam;

void view_thread();
void draw_vehicle_bbox();
void draw_parking_slot(const Eigen::MatrixXd &data);
UdpData data;
std::mutex mutex;

int main()
{
  int sock = socket(AF_INET, SOCK_DGRAM, 0);

  sockaddr_in my_addr{};
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(8088);
  my_addr.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (sockaddr *)&my_addr, sizeof(my_addr));

  char buffer[1500];
  sockaddr_in sender_addr{};
  socklen_t addr_len = sizeof(sender_addr);

  std::thread th(&view_thread);
  th.detach();

  while (true)
  {
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0,
                           (sockaddr *)&sender_addr, &addr_len);
    std::cout << "Received data from target ip\n";

    char sender_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, sizeof(sender_ip));
    int sender_port = ntohs(sender_addr.sin_port);

    std::string expected_ip = "192.168.2.11"; // 你想接收的指定 IP

    if (expected_ip == sender_ip)
    {
      UdpData *recv_data = reinterpret_cast<UdpData *>(buffer);
      {
        std::cout << "@@@@@@@\n";
        std::lock_guard<std::mutex> lck(mutex);
        data = *recv_data;
      }
    }
    else
    {
      std::cout << "❌ Ignoring data from unknown IP\n";
    }
  }

  close(sock);
  return 0;
}

void view_thread()
{
  pangolin::CreateWindowAndBind("APA Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    UdpData draw_data;
    {
      std::lock_guard<std::mutex> lock(mutex);
      draw_data = data;
    }

    Eigen::Vector3d twb(draw_data.pose[0], draw_data.pose[1], .0);
    Eigen::AngleAxisd axang(draw_data.pose[2], Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d Rwb = axang.toRotationMatrix();
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.topLeftCorner(3, 3) = Rwb;
    Twb.topRightCorner(3, 1) = twb;

    for (int i = 0; i < draw_data.slot_num; ++i) {
      Eigen::Map<Eigen::Matrix<float, 2, 4, Eigen::ColMajor>> slot_data(&(draw_data.slot_corners[8 * i]));
      Eigen::MatrixXd slot = slot_data.cast<double>();
      draw_parking_slot(slot);
    }

    glPushMatrix();
    glMultMatrixd(Twb.data());
    draw_vehicle_bbox();
    glPopMatrix();

    pangolin::FinishFrame();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void draw_vehicle_bbox()
{
  glColor3f(0.5, 0.0, 0.5);
  glLineWidth(3.0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glEnd();

  glBegin(GL_LINE_STRIP);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, 0);
  glVertex3f(VEHICLE_FORWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, -0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_FORWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glVertex3f(VEHICLE_BACKWARD_DIST, 0.5 * VEHICLE_WIDTH, VEHICLE_HEIGHT);
  glEnd();
}

void draw_parking_slot(const Eigen::MatrixXd &data)
{

  glPointSize(8.0);

  glBegin(GL_POINTS);
  glColor3f(.0f, 1.0f, 0.0f);
  glVertex3f(data.col(0).x(), data.col(0).y(), .0f);
  glVertex3f(data.col(1).x(), data.col(1).y(), .0f);
  glVertex3f(data.col(2).x(), data.col(2).y(), .0f);
  glVertex3f(data.col(3).x(), data.col(3).y(), .0f);
  glEnd();

  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  Eigen::Vector2d pt = (data.col(0).head(2) + data.col(1).head(2)) * .5f;
  glVertex3f(pt.x(), pt.y(), .0f);
  glEnd();

  glColor3f(.0f, .0f, 1.0f);
  glLineWidth(2.0);
  glBegin(GL_LINE_STRIP);

  glVertex3f(data.col(1).x(), data.col(1).y(), .0f);
  glVertex3f(data.col(2).x(), data.col(2).y(), .0f);
  glVertex3f(data.col(3).x(), data.col(3).y(), .0f);
  glVertex3f(data.col(0).x(), data.col(0).y(), .0f);

  glEnd();

}
