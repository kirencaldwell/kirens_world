#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <vector>
#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace KirensWorld {
namespace Dynamics {

class RigidBodyInterface {
  public:
    RigidBodyInterface();
    RigidBodyInterface(MatrixXd mass, VectorXd initial_state, VectorXd initial_state_dot) :
      _mass(mass), _state(initial_state), _state_dot(initial_state_dot) {};

    void setMass(MatrixXd mass) {
      _mass = mass;
    };
    MatrixXd getMass() {
      return _mass;
    };
    void setState(VectorXd state) {
      _state = state;
      _state_tlm.push_back(_state);
    };
    void setStateDot(VectorXd state_dot) {
      _state_dot = state_dot;
    };
    VectorXd getState() {
      return _state;
    }
    VectorXd getStateDot() {
      return _state_dot;
    };
    void setForces(VectorXd forces) {
      _forces = forces;
    };
    VectorXd getForces() {
      return _forces;
    };
    std::vector<VectorXd> getTelemetry() {
      return _state_tlm;
    };
    void setWorldIndices(std::vector<int> idx) {
      _world_idx = idx;
    };
    std::vector<int> getWorldIndices() {
      // TODO: make this an error
      if (_world_idx.empty()) {
        std::cout << 
          "Index hasnt been set yet, did you forget to add to the world?" 
          << std::endl;
        return {0};
      }
      return _world_idx;
    };

  private:
    MatrixXd _mass;
    VectorXd _state;
    VectorXd _state_dot;
    // TODO: parameterize the size of this vector
    VectorXd _forces = VectorXd::Zero(3);
    std::vector<VectorXd> _state_tlm;
    std::vector<int> _world_idx;

};

class RigidBody : public RigidBodyInterface {
  public:
    RigidBody();
    RigidBody(MatrixXd mass, VectorXd initial_state, VectorXd initial_state_dot) : 
      RigidBodyInterface(mass, initial_state, initial_state_dot) {};

  private:

};

}
}

#endif
