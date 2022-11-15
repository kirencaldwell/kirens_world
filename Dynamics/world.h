#ifndef WORLD_H 
#define WORLD_H 

#include <vector>
#include <map>
#include <memory>
#include "Eigen/Dense"
#include <iostream>
#include <mutex>
#include <thread>

#include "Dynamics/rigid_body.h"
#include "Dynamics/constraint.h"
#include "Dynamics/utilities.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace KirensWorld {
namespace Dynamics {

class SharedJacobian {
  public:
    SharedJacobian(int n_constraints, int n_states): _J(n_constraints, n_states) {
    };
    void zero(int n, int m) {
      //std::lock_guard<std::mutex> g(_mut);
      _J = MatrixXd::Zero(n, m);
      _C = VectorXd::Zero(n);
      std::cout << "J = " << _J << std::endl;
    };

    MatrixXd getJacobian() {
      std::lock_guard<std::mutex> g(_mut);
      return _J;
    };
    void addJacobianTerms(VectorXd J_i, int idx_constraint, std::vector<int> idx) {
      std::lock_guard<std::mutex> g(_mut);
      _J(idx_constraint, idx) = J_i;
    };
    void addConstraintValue(double C_i, int idx_constraint) {
      std::lock_guard<std::mutex> g(_mut);
      _C(idx_constraint) = C_i;
    }
    VectorXd getConstraintValue() {
      return _C;
    }

  private:
    std::mutex _mut;
    MatrixXd _J;
    VectorXd _C;
};

class World {
  public:
    World() {};
    void init();
    void setGravity(VectorXd gravity) {
      _gravity = gravity;
    };
    void addRigidBody(std::shared_ptr<RigidBodyInterface> new_body);
    VectorXd getState() {
      return _state;
    };
    VectorXd getStateDot() {
      return _state_dot;
    };
    void addConstraint(std::unique_ptr<Constraint> new_constraint);
    void updateWorld(double time);

  private:
    VectorXd _gravity = VectorXd::Zero(N_DIMS);
    std::vector<std::shared_ptr<RigidBodyInterface>> _rigid_bodies;
    std::vector<std::unique_ptr<Constraint>> _constraints;
    int _n_constraints = 0;
    int _n_idx = 0;
    VectorXd _state;
    VectorXd _state_dot;
    double _time = 0.0;
    std::shared_ptr<SharedJacobian> _constraint_jacobian;
    std::shared_ptr<SharedJacobian> _constraint_jacobian_rate;
    double _K = 0.001;
    double _B = 0.0001;
    // std::vector<std::thread> _jacobian_threads;

    void updateJacobians(
        Constraint* constraint, 
        int idx_constraint, 
        std::shared_ptr<SharedJacobian> jacobian_data);
    void updateJacobianRates(
        Constraint* constraint, 
        int idx_constraint, 
        std::shared_ptr<SharedJacobian> jacobian_data);

};
}
}
#endif
