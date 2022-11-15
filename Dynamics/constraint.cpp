#include "Dynamics/constraint.h"
#include <iostream>

namespace KirensWorld {
namespace Dynamics {

double NumericalConstraint::computeValue() {
  VectorXd state_a = _body_a->getState();
  if (!_body_b) {
    // This must mean only one rigid body is constrained
    return _computeValue(state_a);
  }
  else {
    VectorXd state_b = _body_b->getState();
    return _computeValue(state_a, state_b);
  }
}

double NumericalConstraint::_computeRate(const VectorXd &state_a) {
  ConstraintJacobianInfo J = _computeJacobian(state_a);
  double C_dot  = J.J[0].dot(_body_a->getStateDot());
  return C_dot;
}

double NumericalConstraint::_computeRate(const VectorXd &state_a, const VectorXd &state_b) {
  ConstraintJacobianInfo J = _computeJacobian(state_a, state_b);
  double C_dot  = J.J[0].dot(_body_a->getStateDot()) +
                  J.J[1].dot(_body_b->getStateDot());
  return C_dot;
}

double NumericalConstraint::computeRate() {
  ConstraintJacobianInfo J = computeJacobian();
  double C_dot  = J.J[0].dot(_body_a->getStateDot()) +
                  J.J[1].dot(_body_b->getStateDot());
  return C_dot;
}

ConstraintJacobianInfo NumericalConstraint::computeJacobian() {
  VectorXd state_a = _body_a->getState();
  if (!_body_b) {
    return _computeJacobian(state_a);
  }
  else {
    VectorXd state_b = _body_b->getState();
    return _computeJacobian(state_a, state_b);
  }
}

ConstraintJacobianInfo NumericalConstraint::_computeJacobian(const VectorXd &state_a) {
  std::vector<int> idx_a = _body_a->getWorldIndices();

  VectorXd Ja(state_a.rows());
  double C0 = _computeValue(state_a);

  VectorXd state_ap = state_a;
  for (int i = 0; i < state_a.rows(); i++) {
    state_ap = state_a;
    state_ap(i) = state_ap(i) + _h;
    double Cp = _computeValue(state_ap);
    Ja(i) = (Cp - C0) / _h;
  }

  ConstraintJacobianInfo data;
  data.J.push_back(Ja);
  data.idx.push_back(idx_a);
  data.C = C0;

  return data;
}

ConstraintJacobianInfo NumericalConstraint::_computeJacobian(const VectorXd &state_a, const VectorXd &state_b) {
  std::vector<int> idx_a = _body_a->getWorldIndices();
  std::vector<int> idx_b = _body_b->getWorldIndices();

  VectorXd Ja(state_a.rows());
  VectorXd Jb(state_b.rows());
  double C0 = _computeValue(state_a, state_b);

  VectorXd state_ap = state_a;
  VectorXd state_bp = state_b;
  for (int i = 0; i < state_a.rows(); i++) {
    state_ap = state_a;
    state_ap(i) = state_ap(i) + _h;
    double Cp = _computeValue(state_ap, state_b);
    Ja(i) = (Cp - C0) / _h;
  }
  for (int i = 0; i < state_b.rows(); i++) {
    state_bp = state_b;
    state_bp(i) = state_bp(i) + _h;
    double Cp = _computeValue(state_a, state_bp);
    Jb(i) = (Cp - C0) / _h;
  }

  ConstraintJacobianInfo data;
  data.J.push_back(Ja);
  data.J.push_back(Jb);
  data.idx.push_back(idx_a);
  data.idx.push_back(idx_b);
  data.C = C0;

  return data;
}

ConstraintJacobianInfo NumericalConstraint::computeJacobianRate() {
  ConstraintJacobianInfo data;
  std::vector<int> idx_a = _body_a->getWorldIndices();
  VectorXd state_a = _body_a->getState();
  VectorXd Ja(state_a.rows());

  VectorXd state_ap = state_a;
  double C0 = _computeRate(state_a);
  if (!_body_b) {
    for (int i = 0; i < state_a.rows(); i++) {
      state_ap = state_a;
      state_ap(i) = state_ap(i) + _h;
      double Cp = _computeRate(state_ap);
      Ja(i) = (Cp - C0) / _h;
    }
    data.J.push_back(Ja);
    data.idx.push_back(idx_a);
  }

  else {
    std::vector<int> idx_b = _body_b->getWorldIndices();
    VectorXd state_b = _body_b->getState();

    VectorXd Jb(state_b.rows());
    double C0 = _computeRate(state_a, state_b);
    VectorXd state_bp = state_b;
    for (int i = 0; i < state_a.rows(); i++) {
      state_ap = state_a;
      state_ap(i) = state_ap(i) + _h;
      double Cp = _computeRate(state_ap, state_b);
      Ja(i) = (Cp - C0) / _h;
    }
    for (int i = 0; i < state_b.rows(); i++) {
      state_bp = state_b;
      state_bp(i) = state_bp(i) + _h;
      double Cp = _computeRate(state_a, state_bp);
      Jb(i) = (Cp - C0) / _h;
    }

    data.J.push_back(Ja);
    data.J.push_back(Jb);
    data.idx.push_back(idx_a);
    data.idx.push_back(idx_b);

  }
  return data;
}

}
}
