#include <gtest/gtest.h>
#include <iostream>
#include <memory>

#include "Dynamics/rigid_body.h"
#include "Dynamics/world.h"
#include "Dynamics/constraint.h"
#include "Dynamics/ball_joint.h"
#include "matplotlib/matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace KirensWorld {
namespace Dynamics {

TEST(WorldTest, BasicFunctions) {

  World kirens_world;
  VectorXd x0(3);
  x0 << 2, 0, 0;
  VectorXd x1(3);
  x1 << 4, 0, 0;

  std::shared_ptr<RigidBodyInterface> test_body(
      new RigidBody(MatrixXd::Identity(3, 3), x0, VectorXd::Zero(3)));
  std::shared_ptr<RigidBodyInterface> test_body2(
      new RigidBody(MatrixXd::Identity(3, 3), x1, VectorXd::Zero(3)));

  VectorXd r0(2);
  r0 << -1, 0;
  VectorXd r1(2);
  r1 << 1, 0;
  VectorXd r2(2);
  r2 << -1, 0;

  std::unique_ptr<BallJointToWorld> ball_joint_to_world(
      new BallJointToWorld(test_body, r0));
  std::unique_ptr<BallJoint> ball_joint(
      new BallJoint(test_body, r1, test_body2, r2));

  kirens_world.addRigidBody(test_body);
  kirens_world.addRigidBody(test_body2);
  kirens_world.addConstraint(std::move(ball_joint_to_world));
  kirens_world.addConstraint(std::move(ball_joint));

  VectorXd gravity(3);
  gravity << 0, -2, 0;
  kirens_world.setGravity(gravity);
  kirens_world.init();

  std::vector<int> idx = test_body->getWorldIndices();
  std::vector<int> idx2 = test_body2->getWorldIndices();

  EXPECT_EQ(idx.front(), 0);
  EXPECT_EQ(idx.back(), 2);
  EXPECT_EQ(idx2.front(), 3);
  EXPECT_EQ(idx2.back(), 5);


  double time = 0;
  double dt = 0.0001;
  std::vector<double> x = {0, 1};
  std::vector<double> y = {0, 1};
  plt::plot(x, y);
  plt::show();
  for (int i = 0; i < 100; i++) {
    time += dt;
    kirens_world.updateWorld(time);
  }

  std::cout << "R0 = " << test_body->getState()({0, 1}).norm() << std::endl;
  std::cout << "R1 = " << (test_body2->getState()({0, 1}) - test_body->getState()({0, 1})).norm() << std::endl;
}

}
}
