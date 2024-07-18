#include "utils/cv.hpp"
#include "utils/glog.hpp"
#include "utils/indicators.hpp"
#include "utils/math.hpp"
#include "utils/pangolin.hpp"

namespace pangolin {


void draw_imu(OpenGlMatrix &Twc, float w) {
  float h = w * 0.75, z = w * 0.6;
  float pts[4][3] = {{-w, h, z},
                     {w,  h, z},
                     {w,  h, -z},
                     {-w, h, -z}};

  glPushMatrix();
#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glBegin(GL_LINES);
  for (int i = 0; i < 4; ++i) {
    glVertex3f(0, 0, 0);
    glVertex3fv(pts[i]);
  }
  glVertex3fv(pts[0]);
  for (int i = 1; i < 4; ++i) {
    glVertex3fv(pts[i]);
    glVertex3fv(pts[i]);
  }
  glVertex3fv(pts[0]);

  glEnd();
  glPopMatrix();
}


void plot_trajectory(YAML::Node cfg,
                     std::vector<double> &vTsImg, std::vector<std::string> &vImgFiles,
                     std::vector<double> &vTsPose, std::vector<Sophus::SE3f> &vPoses) {
  glog::Timer timer;
  cv::GrayLoader grayloader;
  math::ValueSlicer<double> slicer(&vTsPose);

  int delay = 1000 / cfg["fps"].as<int>();
  auto imu_size = cfg["imu_size"].as<float>();
  auto queue_size = cfg["queue_size"].as<int>();
  auto sample_stride = cfg["sample_stride"].as<int>();

  Figure::Ptr pgl_fig = Figure::from_yaml(cfg);
  auto pbar = indicators::getProgressBar(vPoses.size());

  for (int i = 0; i < vPoses.size(); i++) vPoses[i] = vPoses[i].inverse();
  for (int i = 0; i < vTsImg.size() && pgl_fig->is_running(); ++i) {
    timer.reset();
    auto [_, j] = slicer(vTsImg[i]);
    pgl_fig->clear();

    OpenGlMatrix T_world_imu(vPoses[j].matrix());
    pgl_fig->follow(T_world_imu);
    // 绘制关键帧
    glColor3f(0.0, 1.0, 0.0);
    for (int k = MAX(0, int(j) - queue_size * sample_stride); k < j; k += sample_stride) {
      OpenGlMatrix T_wi(vPoses[k].matrix());
      draw_imu(T_wi, imu_size);
    }
    // 绘制当前帧
    glColor3f(0.0, 0.0, 1.0);
    draw_imu(T_world_imu, imu_size);

    pgl_fig->draw();
    pbar.tick();
    cv::imshow("Trajectory", grayloader(vImgFiles[i]));

    int cost = timer.count() * 1e3;
    indicators::set_desc(pbar, "FPS=" + std::to_string(1000 / MAX(delay, cost)), false);
    cv::waitKey(MAX(1, delay - cost));
  }
}


Figure::Figure(std::string window_title, int w, int h) : w(w), h(h), mRender() {
  CreateWindowAndBind(window_title, w, h);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  mDisplay = CreateDisplay().SetHandler(new Handler3D(mRender)).SetBounds(0.0, 1.0, 0.0, 1.0);
}


Figure::Ptr Figure::from_yaml(const YAML::Node &cfg) {
  auto size = YAML::toVec<int>(cfg["resolution"]);
  auto view_point = YAML::toVec<float>(cfg["view_point"]);

  auto pFig = std::make_shared<Figure>(cfg["title"].as<std::string>(), size[0], size[1]);
  pFig->set_focal(cfg["camera_focal"].as<float>());
  pFig->set_view_point(view_point[0], view_point[1], view_point[2],
                       static_cast<pangolin::AxisDirection>(cfg["view_up_axis"].as<int>()));
  return pFig;
}

}
