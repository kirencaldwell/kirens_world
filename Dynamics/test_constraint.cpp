#include <gtest/gtest.h>
#include <iostream>
#include <memory>

#include "Dynamics/rigid_body.h"
#include "Dynamics/world.h"
#include "Dynamics/constraint.h"
#include "Dynamics/ball_joint.h"


namespace KirensWorld {
namespace Dynamics {

TEST(ConstraintTest, BasicFunctions) {
  World kirens_world;
  std::shared_ptr<RigidBodyInterface> test_body(new RigidBody(MatrixXd::Identity(3, 3), VectorXd::Zero(3), VectorXd::Zero(3)));
  std::shared_ptr<RigidBodyInterface> test_body2(new RigidBody(MatrixXd::Identity(3, 3), VectorXd::Zero(3), VectorXd::Zero(3)));


  BallJoint ball_joint(test_body, VectorXd::Ones(2), test_body2, VectorXd::Ones(2));


  kirens_world.addRigidBody(test_body, "bob");
  kirens_world.addRigidBody(test_body2, "bob2");
  kirens_world.init();

  double C = ball_joint.computeValue();
  EXPECT_EQ(C, 0.0);
  double C_dot = ball_joint.computeRate();
  std::cout << "C_dot = " << C_dot << std::endl;

  ConstraintJacobianInfo J = ball_joint.computeJacobian();
  std::cout << J.J_a << std::endl;

  ConstraintJacobianInfo J_dot = ball_joint.computeJacobianRate();
  std::cout << J_dot.J_a << std::endl;
}

}
}
