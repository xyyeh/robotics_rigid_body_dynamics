/**
 * \brief An example app to demonstrate functionalities of the exp_class
 *
 */

#include <robotics/rigid_body_dynamics/config.h>
#include <robotics/rigid_body_dynamics/rigid_body_dynamics.h>

#include <iostream>

int main() {
  std::cout << "Version: ";
  std::cout << ROBOTICS_RIGID_BODY_DYNAMICS_VERSION_MAJOR << ".";
  std::cout << ROBOTICS_RIGID_BODY_DYNAMICS_VERSION_MINOR << ".";
  std::cout << ROBOTICS_RIGID_BODY_DYNAMICS_VERSION_PATCH << std::endl;

  robotics::rigid_body_dynamics::RBDyn rb;
  rb.CreateModel("../resource/FrankaEmikaSubChain.urdf");

  Eigen::VectorXd q(rb.Dof());
  Eigen::VectorXd dq(rb.Dof());

  q << 1, 2, 3, 4;
  dq << 5, 6, 7, 8;

  std::cout << "First, update states before running forward kinematics"
            << std::endl;
  rb.SetQ(q);
  rb.SetDq(dq);
  rb.ForwardKinematics();

  std::cout << "Next, compute the required parameters using RBDyn" << std::endl;
  // configurations
  Eigen::Matrix3d R;
  R << 0.5403023, -0.841471, 0, 0.841471, 0.5403023, 0, 0, 0, 1;
  Eigen::Vector3d p(1, 2, 3);

  // tests
  std::string name{"panda_link4"};

  // declarations
  Eigen::MatrixXd T(4, 4);

  // pose
  rb.EndEffPose(T, name);
  std::cout << T << std::endl;

  std::cout << "See test folder for computations of other values" << std::endl;
}