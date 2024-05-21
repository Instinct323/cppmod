#ifndef ZJSLAM__EXTENSION__STD_HPP
#define ZJSLAM__EXTENSION__STD_HPP

#include <Eigen/Core>

namespace std {


// std::vector -> Eigen
template<typename T>
Eigen::Vector<T, -1> toEigen(const vector<T> &vec) {
  Eigen::Vector<T, -1> v(vec.size());
  for (int i = 0; i < vec.size(); i++) v(i) = vec[i];
  return v;
}

}

#endif
