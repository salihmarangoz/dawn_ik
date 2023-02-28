#ifndef __SALIH_MARANGOZ_SHAPES_H__
#define __SALIH_MARANGOZ_SHAPES_H__

#include <Eigen/Dense>

namespace salih_marangoz_thesis
{

template <typename T>
struct Sphere
{
    Eigen::Matrix<T,3,1> pos;
    float radius;

    void distance(const Sphere<T>& other, T& out)
    {
        out = (other.pos - pos).norm() - other.radius - radius;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace salih_marangoz_thesis

#endif //__SALIH_MARANGOZ_SHAPES_H__