#ifndef IK_MATRICES_H
#define IK_MATRICES_H

#include <Eigen/Dense>

struct Mat4 {
  double a0,  a1,   a2,  a3,
         a4,  a5,   a6,  a7,
         a8,  a9,  a10, a11,
         a12, a13, a14, a15;
};


template<typename T>
inline void matToEigen(const Mat4& m, Eigen::Matrix<T,4,4>& m_out) {
    m_out << T(m.a0),  T(m.a4),  T(m.a8),  T(m.a12),
             T(m.a1),  T(m.a5),  T(m.a9),  T(m.a13),
             T(m.a2),  T(m.a6),  T(m.a10), T(m.a14),
             T(m.a3),  T(m.a7),  T(m.a11), T(m.a15);
}


struct Point2D {
  double x, y;
  Point2D() {}
  Point2D(double x_, double y_) : x(x_), y(y_) {}
};


struct Point3D {
  double x, y, z;
  Point3D() {}
  Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

#endif