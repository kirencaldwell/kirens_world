#include "Dynamics/ball_joint.h"
#include "Dynamics/utilities.h"

namespace KirensWorld {
namespace Dynamics {

double BallJoint::_computeValue(const VectorXd &state_a, const VectorXd &state_b) {
  VectorXd pos_a = state_a(POS_IDX);
  VectorXd pos_b = state_b(POS_IDX);
  VectorXd rot_a = state_a(ROT_IDX);
  VectorXd rot_b = state_b(ROT_IDX);

  VectorXd r_a = pos_a + Utilities::rotateVector(rot_a)*_joint_loc_a;
  VectorXd r_b = pos_b + Utilities::rotateVector(rot_b)*_joint_loc_b;

  return 0.5*(r_a-r_b).dot(r_a-r_b);
}

double BallJointToWorld::_computeValue(const VectorXd &state_a) {
  VectorXd pos_a = state_a(POS_IDX);
  VectorXd rot_a = state_a(ROT_IDX);

  VectorXd r_a = pos_a + Utilities::rotateVector(rot_a)*_joint_loc;

  return 0.5*r_a.dot(r_a);

}
}
}
