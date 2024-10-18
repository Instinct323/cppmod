
#include "utils/glog.hpp"

#include <utils/rs.hpp>

int main(int argc, char **argv) {
#ifdef USE_XSRV
  putenv("DISPLAY=host.docker.internal:0");
#endif
  glog::Logger logger(argv);

  rs2::context ctx;
  LOG(INFO) << "There are " << ctx.query_devices().size() << " connected RealSense devices.";

  rs2::pipeline pipe(ctx);
  pipe.start();

  while (true) {
    rs2::frameset frames = pipe.wait_for_frames();

    rs2::frame color = frames.get_color_frame();
    rs2::frame depth = frames.get_depth_frame();

    cv::Mat color_mat = rs2::toCvMat(color);
    cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
    cv::Mat depth_mat = rs2::toCvMat(depth);

    cv::imshow("color", color_mat);
    cv::imshow("depth", depth_mat);
    if (cv::waitKey(1) == 27) break;
  }

  return 0;
}
