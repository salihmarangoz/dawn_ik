#ifndef IK_TRANSFORMS_H
#define IK_TRANSFORMS_H

#include <Eigen/Dense>
#include <iostream>

#include "ceres/rotation.h"
#include "matrices.h"

using namespace std;

class Transform {
private:
    virtual void _() {}
public:
    template<typename T> void apply( const T* const robot_state,
                                     const Eigen::Matrix<T,4,1> v_in,
                                     Eigen::Matrix<T,4,1> &v_out );
};


class StaticTransform : public Transform {
private:
    Mat4 mat_;
public:
    StaticTransform(const Mat4 mat) : mat_(mat) {}
    template<typename T> void apply( const T* const robot_state,
                                     const Eigen::Matrix<T,4,1> v_in,
                                     Eigen::Matrix<T,4,1> &v_out ) {
        Eigen::Matrix<T,4,4> mat_tmp;
        matToEigen(mat_, mat_tmp);
        v_out = mat_tmp * v_in;
    }
};


class JointTransform : public Transform {
private:
    int joint_idx_;
public:
    JointTransform(int joint_idx) : joint_idx_(joint_idx) {}

    template<typename T> void apply( const T* const robot_state,
                                     const Eigen::Matrix<T,4,1> v_in,
                                     Eigen::Matrix<T,4,1> &v_out ) {
        // apply joint rotation
        // URDF -> OpenRave Collada -> Three.js always results in joint axis = [0, 1, 0]
        T angle_axis[3];
        angle_axis[0] = T(0.0);
        angle_axis[1] = T(robot_state[joint_idx_]) * T(M_PI / 180.0);  // deg to rad
        angle_axis[2] = T(0.0);
        // convert
        T v_in_raw[3];
        v_in_raw[0] = v_in(0) / v_in(3);
        v_in_raw[1] = v_in(1) / v_in(3);
        v_in_raw[2] = v_in(2) / v_in(3);
        // apply rotation
        T v_out_raw[3];
        ceres::AngleAxisRotatePoint<T>(angle_axis, v_in_raw, v_out_raw);
        // convert
        v_out(0) = v_out_raw[0];
        v_out(1) = v_out_raw[1];
        v_out(2) = v_out_raw[2];
        v_out(3) = T(1.0);
    }
};


template<typename T> void Transform::apply( const T* const robot_state,
                                            const Eigen::Matrix<T,4,1> v_in,
                                            Eigen::Matrix<T,4,1> &v_out ) {
    if (StaticTransform* trans = dynamic_cast<StaticTransform*>(this)) {
        trans->apply(robot_state, v_in, v_out);
    }
    else if (JointTransform* trans = dynamic_cast<JointTransform*>(this)) {
        trans->apply(robot_state, v_in, v_out);
    }
    else {
        cout << "dynamic_cast is failing" << endl;
    }
}

#endif