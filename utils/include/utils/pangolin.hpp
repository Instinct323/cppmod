#ifndef UTILS__PANGOLIN_HPP
#define UTILS__PANGOLIN_HPP

#define USE_EIGEN

#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

#include "file.hpp"
#include "math.hpp"

namespace pangolin {

class Trajectory;

// 绘制相机模型
void draw_imu(const OpenGlMatrix &Tcw, float w);

// 绘制轨迹
void plot_trajectory(const YAML::Node cfg,
                     const std::vector<double> &vTsImg,
                     const std::vector<std::string> &vImgFiles,
                     Trajectory &trace);


class Trajectory {

public:
    const float imu_size;
    const int sample_stride;
    const int trail_size;
    const Eigen::Vector3f lead_color, trail_color;

    std::vector<OpenGlMatrix> vTcw;
    math::ValueSlicer<double> slicer;

    explicit Trajectory(YAML::Node cfg, std::vector<double> &vTimestamp, std::vector<Sophus::SE3f> &vTwc
    ) : slicer(&vTimestamp), imu_size(cfg["imu_size"].as<float>()),
        sample_stride(cfg["sample_stride"].as<int>()), trail_size(cfg["trail_size"].as<int>()),
        lead_color(YAML::toEigen<float>(cfg["lead_color"])), trail_color(YAML::toEigen<float>(cfg["trail_color"])) {
        vTcw.reserve(vTwc.size());
        for (int i = 0; i < vTwc.size(); ++i) vTcw.emplace_back(vTwc[i].inverse().matrix());
    }

    const OpenGlMatrix& plot(double t) {
        auto [_, i] = slicer(t);
        // 绘制轨迹
        glColor3f(trail_color[0], trail_color[1], trail_color[2]);
        for (int j = std::max(0, i - trail_size * sample_stride); j < i; j += sample_stride) draw_imu(vTcw[j], imu_size);
        // 绘制当前帧
        glColor3f(lead_color[0], lead_color[1], lead_color[2]);
        draw_imu(vTcw[i], imu_size);
        return vTcw[i];
    }
};


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

    void set_panel(float radio) {
        if (radio > 0) CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, radio);
    }

    inline bool is_running() { return !ShouldQuit(); }

    inline void clear() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void follow(const OpenGlMatrix &Twc) {
        mRender.Follow(Twc);
        mDisplay.Activate(mRender);
    }

    inline void draw() { FinishFrame(); }
};

}

#endif
