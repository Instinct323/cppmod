#ifndef ZJSLAM__CAMERA__FISHEYE_HPP
#define ZJSLAM__CAMERA__FISHEYE_HPP

#include "base.hpp"


class KannalaBrandt8 : public CameraBase {
    // 序列化
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & boost::serialization::base_object<CameraBase>(*this);
    }

protected:
    // mvParam: [fx, fy, cx, cy, k0, k1, k2, k3]
    static int nParam;

public:
    KannalaBrandt8() { mvParam.resize(nParam); }

    KannalaBrandt8(const std::vector<float> &vParam) : CameraBase(vParam) { assert(mvParam.size() == nParam); }

    CameraType getType() { return CameraType::FISHEYE; }
};

#endif
