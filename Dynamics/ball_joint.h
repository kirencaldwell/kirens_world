#ifndef BALL_JOINT_H 
#define BALL_JOINT_H 

#include <vector>
#include <map>
#include <memory>
#include "Eigen/Dense"
#include <iostream>

#include "Dynamics/rigid_body.h"
#include "Dynamics/constraint.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace KirensWorld {
namespace Dynamics {

class BallJoint : public NumericalConstraint {
  public:
    BallJoint(std::shared_ptr<RigidBodyInterface> body_a, VectorXd joint_loc_a,
              std::shared_ptr<RigidBodyInterface> body_b, VectorXd joint_loc_b):
      NumericalConstraint(body_a, body_b, 0.0001), 
      _joint_loc_a(joint_loc_a),
      _joint_loc_b(joint_loc_b) {};

  private:
    VectorXd _joint_loc_a;
    VectorXd _joint_loc_b;
    double _computeValue(const VectorXd &state_a, const VectorXd &state_b) override;

};

class BallJointToWorld : public NumericalConstraint {
  public:
    BallJointToWorld(std::shared_ptr<RigidBodyInterface> body, VectorXd joint_loc):
    NumericalConstraint(body, 0.0001),
    _joint_loc(joint_loc) {};

  private:
    VectorXd _joint_loc;
    double _computeValue(const VectorXd &state_a) override;

};

}
}
#endif
