#include "utils/glog.hpp"
#include "utils/rs2.hpp"

int main(int argc, char **argv) {
#ifdef USE_XSRV
  putenv("DISPLAY=host.docker.internal:0");
#endif
  glog::Logger logger(argv);

  rs2::context ctx;
  rs2::devices_profile();

  rs2::pipeline pipe(ctx);
  pipe.start();
  rs2::frames_profile(pipe.wait_for_frames());


  while (true) {
    rs2::frameset frames = pipe.wait_for_frames();

    auto acc = frames.first_or_default(RS2_STREAM_ACCEL);
    if (acc) {
      auto acc_data = acc.as<rs2::motion_frame>().get_motion_data();
      LOG(INFO) << "Accel: " << acc_data.x << " " << acc_data.y << " " << acc_data.z;
    }
  }

  return 0;
}
