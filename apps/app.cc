/**
 * \brief An example app to demonstrate functionalities of the exp_class
 *
 */

#include <robotics/exp_proj/config.h>
#include <robotics/exp_proj/exp_class.h>

#include <iostream>

int main() {
  std::cout << "Version: ";
  std::cout << ROBOTICS_EXP_PROJ_VERSION_MAJOR << ".";
  std::cout << ROBOTICS_EXP_PROJ_VERSION_MINOR << ".";
  std::cout << ROBOTICS_EXP_PROJ_VERSION_PATCH << std::endl;

  robotics::exp_proj::ExpClass a;
}