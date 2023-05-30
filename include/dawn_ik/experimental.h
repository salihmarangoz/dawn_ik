#ifndef DAWN_IK_EXPERIMENTAL_H
#define DAWN_IK_EXPERIMENTAL_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

//================ Experiment-1: Relaxed IK Loss ================
namespace ceres {
class RelaxedIKLoss final : public LossFunction {
 public:
  explicit RelaxedIKLoss(double a);
  void Evaluate(double, double*) const override;

 private:
  const double a_;
};
}
//================================================================

#endif // DAWN_IK_EXPERIMENTAL_H