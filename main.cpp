#include <iostream>
#include <memory>

#include "Dynamics/rigid_body.h"
#include "Dynamics/world.h"
#include "Dynamics/constraint.h"
#include "Dynamics/ball_joint.h"
#include "matplotlib/matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace KirensWorld;
using namespace Dynamics;

int main() {

  World kirens_world;
  VectorXd x0(3);
  x0 << 1, 0, 0;
  VectorXd x1(3);
  x1 << 3, 0, 0;
  VectorXd x2(3);
  x2 << 5, 0, 0;

  std::shared_ptr<RigidBodyInterface> test_body(
      new RigidBody(MatrixXd::Identity(3, 3), x0, VectorXd::Zero(3)));
  std::shared_ptr<RigidBodyInterface> test_body2(
      new RigidBody(MatrixXd::Identity(3, 3), x1, VectorXd::Zero(3)));
  std::shared_ptr<RigidBodyInterface> test_body3(
      new RigidBody(MatrixXd::Identity(3, 3), x2, VectorXd::Zero(3)));

  VectorXd r0(2);
  r0 << -1, 0;
  VectorXd r1(2);
  r1 << 1, 0;
  VectorXd r2(2);
  r2 << -1, 0;
  VectorXd r3(2);
  r3 << 1, 0;
  VectorXd r4(2);
  r4 << -1, 0;

  std::unique_ptr<BallJointToWorld> ball_joint_to_world(
      new BallJointToWorld(test_body, r0));
  std::unique_ptr<BallJoint> ball_joint(
      new BallJoint(test_body, r1, test_body2, r2));
  std::unique_ptr<BallJoint> ball_joint2(
      new BallJoint(test_body2, r3, test_body3, r4));

  kirens_world.addRigidBody(test_body);
  kirens_world.addRigidBody(test_body2);
  kirens_world.addRigidBody(test_body3);
  kirens_world.addConstraint(std::move(ball_joint_to_world));
  kirens_world.addConstraint(std::move(ball_joint));
  kirens_world.addConstraint(std::move(ball_joint2));

  VectorXd gravity(3);
  gravity << 0, -1, 0;
  kirens_world.setGravity(gravity);
  kirens_world.init();

  std::vector<int> idx = test_body->getWorldIndices();
  std::vector<int> idx2 = test_body2->getWorldIndices();
  
  std::vector<double> time_vec;
  std::vector<double> x0_tlm;
  std::vector<double> y0_tlm;
  std::vector<double> x1_tlm;
  std::vector<double> y1_tlm;
  std::vector<double> x2_tlm;
  std::vector<double> y2_tlm;
  double time = 0;
  double dt = 0.001;
  double Tf = 15.0;
  int N = std::floor(Tf / dt);

  for (int i = 0; i < N; i++) {
    time_vec.push_back(time);
    time += dt;
    kirens_world.updateWorld(time);
    auto state0 = test_body->getState();
    auto state1 = test_body2->getState();
    auto state2 = test_body3->getState();
    x0_tlm.push_back(state0(0));
    y0_tlm.push_back(state0(1));
    x1_tlm.push_back(state1(0));
    y1_tlm.push_back(state1(1));
    x2_tlm.push_back(state2(0));
    y2_tlm.push_back(state2(1));
  }

  plt::figure(1);
  plt::plot(x0_tlm, y0_tlm);
  plt::plot(x1_tlm, y1_tlm);
  plt::plot(x2_tlm, y2_tlm);

  plt::figure(2);
  plt::plot(time_vec, x0_tlm);
  plt::plot(time_vec, y0_tlm);

  plt::show();

  return 0;
}

