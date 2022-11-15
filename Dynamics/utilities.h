#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>
#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace KirensWorld {
namespace Dynamics {

const std::vector<int> POS_IDX {0, 1};
const std::vector<int> ROT_IDX {2};
const int N_DIMS = POS_IDX.size() + ROT_IDX.size();
const std::vector<int> POS_RATE_IDX {0, 1, 2};
const std::vector<int> ROT_RATE_IDX {3, 4, 5};

namespace Utilities {

//constexpr int N_DIMS = 3;

MatrixXd rotateVector(VectorXd theta);

}
}
}
#endif
