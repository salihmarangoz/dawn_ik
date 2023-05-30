#include <dawn_ik/experimental.h>

//================ Experiment-1: Relaxed IK Loss ================
namespace ceres
{
RelaxedIKLoss::RelaxedIKLoss(double a) : a_(a) {
  CHECK_GE(a, 0.0);
}

// desmos: https://www.desmos.com/calculator/9epwyzkkk7
// based on "RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion" by Daniel Rakita, et. al.
// note: relaxed ^4 to ^2 for the outlier part of the loss. ^4 is crazy...
// more note: relaxed ^2 to l1 for the outlier part. this may be used for something else...
void RelaxedIKLoss::Evaluate(double s, double* rho) const {
    const double c = 0.1; // TODO
    const double r = 0.01; // TODO
    const double s2 = s*s;
    const double c2 = c*c;
    const double c4 = c2*c2;
    const double g = std::exp( -s2/(2*c2) );

    // L2 loss for outliers
    //rho[0] = a_*( -g + r*s2 + 1 );
    //rho[1] = a_*( (s*g) / c2 + 2*r*s );
    //rho[2] = a_*( g/c2 - (s2*g)/c4 + 2*r );

    // L1 loss for outliers
    rho[0] = a_*( -g + r*s + 1 );
    rho[1] = a_*( (s*g) / c2 + r );
    rho[2] = a_*( g/c2 - (s2*g)/c4 );
}
}
//================================================================