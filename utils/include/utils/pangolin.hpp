#ifndef UTILS__PANGOLIN_HPP
#define UTILS__PANGOLIN_HPP

#define USE_EIGEN

#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "file.hpp"

namespace pangolin {

// 绘制相机模型
void draw_imu(OpenGlMatrix &Twc, float w);

// 绘制轨迹
void plot_trajectory(YAML::Node cfg,
                     std::vector<double> &vTsImg, std::vector<std::string> &vImgFiles,
                     std::vector<double> &vTsPose, std::vector<Sophus::SE3f> &vPoses);


class Figure {

public:
    typedef std::shared_ptr<Figure> Ptr;

    const int w, h;
    OpenGlRenderState mRender;
    View mDisplay;

    static Ptr from_yaml(const YAML::Node &cfg);

    Figure(std::string window_title, int w = 640, int h = 480);

    void set_focal(GLprecision focal, GLprecision z_near = 0.1, GLprecision z_far = 1000) {
      mRender.SetProjectionMatrix(ProjectionMatrix(w, h, focal, focal, w / 2, h / 2, z_near, z_far));
    }

    void set_view_point(GLprecision x, GLprecision y, GLprecision z, AxisDirection up) {
      mRender.SetModelViewMatrix(ModelViewLookAt(x, y, z, 0, 0, 0, up));
    }

    inline bool is_running() { return !ShouldQuit(); }

    inline void clear() { glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }

    void follow(OpenGlMatrix &Twc) {
      mRender.Follow(Twc);
      mDisplay.Activate(mRender);
    }

    inline void draw() { FinishFrame(); }
};

}

#endif
