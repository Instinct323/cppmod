#include <cstdlib>
#include <opencv2/opencv.hpp>

#include "utils/glog.hpp"
#include "utils/pangolin.hpp"
#include "zjcv/dataset/tum_vi.hpp"

void pangolin_test();


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  pangolin_test();

  return 0;
}


void pangolin_test() {
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/example/TUM-VI.yaml");
  dataset::TumVI tum_vi(cfg["dataset"].as<std::string>());
  dataset::Timestamps vImgTs, vPoseTs;
  dataset::Filenames vImgLeft;
  dataset::Poses vPose;

  tum_vi.load_image(vImgTs, vImgLeft, "cam0");
  tum_vi.load_pose(vPoseTs, vPose);

  auto fig = pangolin::Figure::from_yaml(cfg["viewer"]);
  pangolin::Trajectory trace(cfg["viewer"], vPoseTs, vPose);
  for (int i = 0; i < vImgTs.size(); ++i) {
    fig->clear();
    const pangolin::OpenGlMatrix &T_imu_world = trace.plot(vImgTs[i]), T_world_imu = T_imu_world.Inverse();
    glLineWidth(2);
    glColor3f(1.f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(1.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 1.f, 0.f);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(T_imu_world(0, 3), T_imu_world(1, 3), T_imu_world(2, 3));
    LOG(INFO) << T_imu_world(0, 3) << " " << T_imu_world(1, 3) << " " << T_imu_world(2, 3);
    glEnd();

    fig->follow(T_imu_world);

    fig->draw();
    cv::imshow("TUM-VI", cv::imread(vImgLeft[i]));
    cv::waitKey(1);
  }
}
