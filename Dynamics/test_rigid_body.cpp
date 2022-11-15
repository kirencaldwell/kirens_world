#include <gtest/gtest.h>
#include <iostream>

#include "Dynamics/rigid_body.h"


namespace KirensWorld {
namespace Dynamics {

TEST(RigidBodyTest, BasicFunctions) {

  RigidBody test_body(MatrixXd::Identity(3, 3), VectorXd::Zero(3), VectorXd::Zero(3));

  std::cout << test_body.getState() << std::endl;
  std::cout << test_body.getStateDot() << std::endl;

}

}
}
