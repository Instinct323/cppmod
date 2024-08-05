#include <cstdlib>
#include <opencv2/opencv.hpp>

#include "utils/glog.hpp"
#include "utils/pangolin.hpp"
#include "zjcv/dataset/tum_vi.hpp"

void pangolin_test();


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  cv::Mat_<uchar> mask(10, 10), out(1, 10, uchar(20));
  LOG(INFO) << mask;
  cv::reduce(mask.rowRange(0, 2), out, 0, cv::REDUCE_MAX);
  LOG(INFO) << out;

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
  LOG(INFO) << vPose[0].matrix3x4();

  auto fig = pangolin::Figure::from_yaml(cfg["viewer"]);
  pangolin::Trajectory trace(cfg["viewer"], vPoseTs, vPose);
  for (int i = 0; i < vImgTs.size(); ++i) {
    fig->clear();
    pangolin::OpenGlMatrix &T_world_imu = trace.plot(vImgTs[i]), Tw = T_world_imu.Inverse();
    fig->follow(T_world_imu);
    glLineWidth(2);
    glColor3f(1.f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(Tw(0, 3), Tw(1, 3), Tw(2, 3));
    glEnd();
    LOG(INFO) << Tw(0, 3) << " " << Tw(1, 3) << " " << Tw(2, 3);
    fig->draw();
    cv::imshow("TUM-VI", cv::imread(vImgLeft[i]));
    cv::waitKey(1);
  }
}
