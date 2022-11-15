#include "Dynamics/world.h"

#include <numeric>


namespace KirensWorld {
namespace Dynamics {

void World::init() {
  if (!_rigid_bodies.empty()) {
    _state = VectorXd::Zero(_n_idx);
    _state_dot = VectorXd::Zero(_n_idx);
  }
  // TODO: this doesnt seem like the right way to do this
  std::shared_ptr<SharedJacobian> constraint_jacobian( new SharedJacobian(_n_constraints, _n_idx));
  std::shared_ptr<SharedJacobian> constraint_jacobian_rate( new SharedJacobian(_n_constraints, _n_idx));
  _constraint_jacobian = constraint_jacobian;
  _constraint_jacobian_rate = constraint_jacobian_rate;

  //for (int i = 0; i < (int)_constraints.size(); i++) {
  //  _jacobian_threads.push_back(
  //      std::thread(
  //        &World::updateJacobians, 
  //        _constraints[i].get(), 
  //        i, 
  //        _constraint_jacobian));
  //}
}

void World::updateJacobianRates(Constraint* constraint, int idx_constraint, std::shared_ptr<SharedJacobian> jacobian_data) {
 
  std::cout << "updateJacobianRates()" << std::endl; 
  ConstraintJacobianInfo jac = constraint->computeJacobianRate();
  for (int i = 0; i < jac.J.size(); i++) {
    jacobian_data->addJacobianTerms(jac.J[i], idx_constraint, jac.idx[i]);
    jacobian_data->addConstraintValue(jac.C, idx_constraint);
  }
  std::cout << "updateJacobianRates() end" << std::endl; 

}
void World::updateJacobians(Constraint* constraint, int idx_constraint, std::shared_ptr<SharedJacobian> jacobian_data) {
 
  std::cout << "updateJacobians()" << std::endl; 
  ConstraintJacobianInfo jac = constraint->computeJacobian();
  for (int i = 0; i < jac.J.size(); i++) {
    jacobian_data->addJacobianTerms(jac.J[i], idx_constraint, jac.idx[i]);
    jacobian_data->addConstraintValue(jac.C, idx_constraint);
  }
  std::cout << "updateJacobians() end" << std::endl; 

}

void World::addRigidBody(std::shared_ptr<RigidBodyInterface> new_body) {
  // Give the rigid body the indices of its states in the world
  std::vector<int> world_idx(N_DIMS);
  std::iota(world_idx.begin(), world_idx.end(), _n_idx);
  _n_idx += N_DIMS;
  new_body->setWorldIndices(world_idx);

  // add rigid body to the world
  _rigid_bodies.push_back(new_body);
}

void World::addConstraint(std::unique_ptr<Constraint> new_constraint) {
  _constraints.push_back(std::move(new_constraint));
  _n_constraints++;
}

void World::updateWorld(double time) {
  double dt = time - _time;
  _time = time;
  std::cout << "t = " << time << std::endl;
  std::cout << "dt = " << dt << std::endl;

  VectorXd states(_n_idx);
  VectorXd states_dot(_n_idx);
  VectorXd external_forces(_n_idx);
  VectorXd constaint_forces(_n_idx);
  MatrixXd mass(_n_idx, _n_idx);

  // Compute Constraint Jacobian and Jacobian rate
  // TODO: should this be multi-threaded?
  _constraint_jacobian->zero(_constraints.size(), _n_idx);
  _constraint_jacobian_rate->zero(_constraints.size(), _n_idx);
  for (int i = 0; i < (int)_constraints.size(); i++) {
    updateJacobians(_constraints[i].get(), i, _constraint_jacobian);
    updateJacobianRates(_constraints[i].get(), i, _constraint_jacobian_rate);
  }
  MatrixXd J = _constraint_jacobian->getJacobian();
  std::cout << "J_tot = " << J << std::endl;
  MatrixXd J_dot = _constraint_jacobian_rate->getJacobian();
  std::cout << "J_dot_tot = " << J_dot << std::endl;
  VectorXd C = _constraint_jacobian->getConstraintValue();
  VectorXd C_dot = _constraint_jacobian_rate->getConstraintValue();
  std::cout << "C = " << C << std::endl;

  // Get all of the rigid body states
  for (auto &body: _rigid_bodies) {
    std::vector<int> idx = body->getWorldIndices();
    states(idx) = body->getState();
    states_dot(idx) = body->getStateDot();
    // TODO: is this the best way of adding gravity?
    external_forces(idx) = body->getForces() + body->getMass()*_gravity;
    mass(idx, idx) = body->getMass();
  }

  std::cout << "states = " << states << std::endl;
  std::cout << "states_dot = " << states_dot << std::endl;
  std::cout << "mass = " << mass << std::endl;

  // doing math
  MatrixXd M_inv = mass.inverse();
  MatrixXd A_temp = J*M_inv*J.transpose();
  VectorXd b_temp = -J_dot*states_dot - J*M_inv*external_forces - _K*C - _B*C_dot;
  VectorXd lagrangian_multiplier = A_temp.inverse()*b_temp;
  VectorXd constraint_forces = J.transpose()*lagrangian_multiplier;
  std::cout << "Fe = " << external_forces << std::endl;
  std::cout << "Fc = " << constraint_forces << std::endl;
  VectorXd accels = M_inv*(constraint_forces + external_forces);
  std::cout << "Accels = " << accels << std::endl;

  // Set the rigid body states
  states = states + dt*states_dot;
  states_dot = states_dot + dt*accels;
  for (auto &body: _rigid_bodies) {
    std::vector<int> idx = body->getWorldIndices();
    body->setState(states(idx));
    body->setStateDot(states_dot(idx));
  }

}

}
}
