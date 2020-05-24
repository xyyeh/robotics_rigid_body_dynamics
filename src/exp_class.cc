#include <iostream>

#include "robotics/exp_proj/exp_class.h"

namespace robotics {
namespace exp_proj {

bool ExpClass::CalcValue() {
  // multiply parameter by two
  parameter_ = 2 * parameter_;
  return true;
}

} // namespace exp_proj

} // namespace robotics
