#ifndef CONSTRAINT_H 
#define CONSTRAINT_H 

#include <vector>
#include <map>
#include <memory>
#include "Eigen/Dense"
#include <iostream>

#include "Dynamics/rigid_body.h"
#include "Dynamics/utilities.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace KirensWorld {
namespace Dynamics {

struct ConstraintJacobianInfo {
  std::vector<VectorXd> J;
  std::vector<std::vector<int>> idx;
  double C;
};

class Constraint {
  public:
    Constraint(std::shared_ptr<RigidBodyInterface> body_a,
        std::shared_ptr<RigidBodyInterface> body_b):
      _body_a(body_a), _body_b(body_b) {};
    Constraint(std::shared_ptr<RigidBodyInterface> body_a):
      _body_a(body_a) {};

    virtual double computeValue() = 0;
    virtual double computeRate() = 0;
    virtual ConstraintJacobianInfo computeJacobian() = 0;
    virtual ConstraintJacobianInfo computeJacobianRate() = 0;

  protected:
    std::shared_ptr<RigidBodyInterface> _body_a;
    std::shared_ptr<RigidBodyInterface> _body_b;
};

class NumericalConstraint : public Constraint {
  public:
    NumericalConstraint(std::shared_ptr<RigidBodyInterface> body_a, std::shared_ptr<RigidBodyInterface> body_b, double h):
      Constraint(body_a, body_b), _h(h) {};
    NumericalConstraint(std::shared_ptr<RigidBodyInterface> body_a, double h):
      Constraint(body_a), _h(h) {};

    double computeRate() override;
    double computeValue() override;
    ConstraintJacobianInfo computeJacobian() override;
    ConstraintJacobianInfo computeJacobianRate() override;

  private:
    double _h;
    ConstraintJacobianInfo _computeJacobian(const VectorXd &state_a, const VectorXd &state_b);
    ConstraintJacobianInfo _computeJacobian(const VectorXd &state_a);
    virtual double _computeValue(
        const VectorXd &state_a, 
        const VectorXd &state_b) {return 0.0;};
    virtual double _computeValue(const VectorXd &state_a) {return 0.0;};
    double _computeRate(const VectorXd &state_a, const VectorXd &state_b);
    double _computeRate(const VectorXd &state_a);

};

}
}

#endif
