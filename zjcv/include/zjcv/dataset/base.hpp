#ifndef ZJCV__DATASET__BASE_HPP
#define ZJCV__DATASET__BASE_HPP

#include <filesystem>
#include <sophus/se3.hpp>

#include "../imu.hpp"
#include "utils/file.hpp"

namespace dataset {

typedef std::vector<double> Timestamps;
typedef std::vector<std::string> Filenames;
typedef std::vector<Sophus::SE3d> Poses;
typedef std::vector<Eigen::Vector3d> Accels;
typedef std::vector<IMU::Sample> IMUsamples;


class Base {

protected:
    std::string mPath;

public:
    explicit Base(const std::string &path) : mPath(path + "/") {}

    Base(const Base &) = delete;

    operator std::string() const { return mPath; }

    friend std::ostream &operator<<(std::ostream &os, const Base &dataset) { return os << dataset.mPath; }
};

}

#endif
