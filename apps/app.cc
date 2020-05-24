/**
 * \brief An example app to demonstrate functionalities of the exp_class
 *
 */

#include <robotics/rigid_body_dynamics/config.h>
#include <robotics/rigid_body_dynamics/exp_class.h>

#include <iostream>

int main() {
  std::cout << "Version: ";
  std::cout << ROBOTICS_RIGID_BODY_DYNAMICS_VERSION_MAJOR << ".";
  std::cout << ROBOTICS_RIGID_BODY_DYNAMICS_VERSION_MINOR << ".";
  std::cout << ROBOTICS_RIGID_BODY_DYNAMICS_VERSION_PATCH << std::endl;

  robotics::rigid_body_dynamics::ExpClass a;
}