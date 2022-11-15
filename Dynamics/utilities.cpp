#include "utilities.h"
#include "Eigen/Geometry"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace KirensWorld {
namespace Dynamics {
namespace Utilities {

MatrixXd rotateVector(VectorXd theta) {

  if (theta.rows() == 1) {
    // 2d rotation
    MatrixXd R(2, 2);
    R << cos(theta[0]), -sin(theta[0]),
         sin(theta[0]), cos(theta[0]);
    return R;
  }
  if (theta.rows() == 4) {
    // quaternion
    Eigen::Quaterniond q(theta[0], theta[1], theta[2], theta[3]);
    return q.toRotationMatrix();
  }
  return MatrixXd::Identity(3, 3);
}

}
}
}
