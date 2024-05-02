#ifndef ZJSLAM__DATASET__BASE_HPP
#define ZJSLAM__DATASET__BASE_HPP

#include <filesystem>
#include <sophus/se3.hpp>

#include "../imu_type.hpp"
#include "../utils.hpp"

namespace dataset {

    class Base {

    protected:
        std::string mPath;

    public:
        typedef std::vector<double> Timestamps;
        typedef std::vector<std::string> Filenames;
        typedef std::vector<Sophus::SE3d> Poses;
        typedef std::vector<Eigen::Vector3d> Accels;
        typedef std::vector<IMU::Sample> IMUsamples;

        explicit Base(const std::string &path) : mPath(path + "/") {}

        friend std::ostream &operator<<(std::ostream &os, const Base &dataset) {
          return os << "Base(" << dataset.mPath << ")";
        }
    };
}

#endif
