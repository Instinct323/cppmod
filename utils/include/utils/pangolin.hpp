#ifndef UTILS__PANGOLIN_HPP
#define UTILS__PANGOLIN_HPP

#include <pangolin/pangolin.h>

namespace pangolin {

// 绘制相机模型
void draw_camera(OpenGlMatrix &Twc, float w);


class Figure {

public:
    int w, h;
    OpenGlRenderState mRender;
    View mDisplay;

    Figure(std::string window_title, int w = 640, int h = 480) : w(w), h(h) {
      glEnable(GL_DEPTH_TEST);
      // glEnable(GL_BLEND);
      // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      CreateWindowAndBind(window_title, w, h);
      mDisplay = CreateDisplay().SetHandler(new Handler3D(mRender));
    }

    operator bool() { return !ShouldQuit(); }

    void clear() { glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }

    void draw() {
      mDisplay.Activate(mRender);
      FinishFrame();
    }
};

}

#endif
