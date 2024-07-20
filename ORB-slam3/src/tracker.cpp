#include "zjcv/slam.hpp"

namespace slam {


void Tracker::process() {
  mpSystem->mpAtlas->mpCurMap->insert_keyframe(mpLastFrame);
  if (!mpRefFrame) mpRefFrame = mpLastFrame;
}

}
