
#include "macros.h"

#include <iostream>
#include <stdexcept>

#include <robotics/rigid_body_dynamics/rigid_body_dynamics.h>

#include <RBDyn/parsers/common.h>

namespace robotics {
namespace rigid_body_dynamics {

void RBDyn::CreateModel(const std::string& urdf_file_path) {
  try {
    rbd::parsers::ParserResult res = rbd::parsers::from_file(urdf_file_path);

    mb_ = res.mb;
    mbc_ = res.mbc;
    mbg_ = res.mbg;

  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }
}

void RBDyn::SetQ(const Eigen::VectorXd& q) {
  for (unsigned int i = 0; i < Dof(); i++) {
    mbc_.q[i + 1][0] = q[i];
  }
}

void RBDyn::SetDq(const Eigen::VectorXd& dq) {
  for (unsigned int i = 0; i < Dof(); i++) {
    mbc_.alpha[i + 1][0] = dq[i];
  }
}

void RBDyn::Q(Eigen::VectorXd& q) {
  ASSERT(q.size() == Dof(), "incorrect size for q.");
  for (unsigned int i = 0; i < Dof(); i++) {
    q(i) = mbc_.q[i + 1][0];
  }
}

void RBDyn::Dq(Eigen::VectorXd& dq) {
  ASSERT(dq.size() == Dof(), "incorrect size for q.");
  for (unsigned int i = 0; i < Dof(); i++) {
    dq(i) = mbc_.alpha[i + 1][0];
  }
}

void RBDyn::ForwardKinematics() {
  rbd::forwardKinematics(mb_, mbc_);
  rbd::forwardVelocity(mb_, mbc_);
}

void RBDyn::EndEffPose(Eigen::MatrixXd& O_T_ee, const std::string& body_name,
                       const Eigen::Matrix3d& ofs_rotation,
                       const Eigen::Vector3d& ofs_position) {
  auto index = BodyIdFromName(body_name);
  ASSERT(index >= 0, body_name << " not found.");

  // construct spatial transformation ee_X_i * i_X_O
  sva::PTransformd ee_X_i(ofs_rotation.transpose(), ofs_position);
  sva::PTransformd ee_X_O = ee_X_i * mbc_.bodyPosW[index];

  // construct O_T_ee
  O_T_ee.resize(4, 4);
  O_T_ee.setIdentity();
  O_T_ee.block<3, 3>(0, 0) = ee_X_O.rotation().transpose();
  O_T_ee.block<3, 1>(0, 3) = ee_X_O.translation();
}

void RBDyn::Jacobian(Eigen::MatrixXd& O_J_ee, const std::string& body_name,
                     const Eigen::Vector3d& ofs_position) {
  auto index = BodyIdFromName(body_name);
  ASSERT(index >= 0, body_name << " not found.");

  rbd::Jacobian jac(mb_, body_name, ofs_position);
  O_J_ee = jac.jacobian(mb_, mbc_);

  // swap spatial notations linear and angular components to [linear; angular]
  O_J_ee.row(0).swap(O_J_ee.row(3));
  O_J_ee.row(1).swap(O_J_ee.row(4));
  O_J_ee.row(2).swap(O_J_ee.row(5));
}

void RBDyn::JacobianDot(Eigen::MatrixXd& O_dJ_ee, const std::string& body_name,
                        const Eigen::Vector3d& ofs_position) {
  auto index = BodyIdFromName(body_name);
  ASSERT(index >= 0, body_name << " not found.");

  rbd::Jacobian jac(mb_, body_name, ofs_position);
  O_dJ_ee = jac.jacobianDot(mb_, mbc_);

  // swap spatial notations linear and angular components to [linear; angular]
  O_dJ_ee.row(0).swap(O_dJ_ee.row(3));
  O_dJ_ee.row(1).swap(O_dJ_ee.row(4));
  O_dJ_ee.row(2).swap(O_dJ_ee.row(5));
}

void RBDyn::MassMatrix(Eigen::MatrixXd& M) {
  ASSERT(M.rows() == M.cols(), "mass matrix is not square");
  ASSERT(M.rows() == Dof(), "incorrect size for mass matrix");

  rbd::ForwardDynamics fd(mb_);
  fd.computeH(mb_, mbc_);
  M = fd.H();
}

void RBDyn::GravityVector(Eigen::VectorXd& G) {
  ASSERT(G.size() == Dof(), "incorrect size for gravity compensation vector");

  rbd::ForwardDynamics fd(mb_);

  // record existing states
  Eigen::VectorXd ori_dq(Dof());
  Dq(ori_dq);

  // zero out dq
  SetDq(Eigen::VectorXd::Zero(Dof()));
  ForwardKinematics();

  // compute C = cor/cent + G where cor/cent @ zero velocity = 0
  fd.computeC(mb_, mbc_);
  G = fd.C();

  // revert states
  SetDq(ori_dq);
  ForwardKinematics();
}

void RBDyn::CoriolisVector(Eigen::VectorXd& C) {
  ASSERT(C.size() == Dof(), "incorrect size for coriolis/centrifugal vector");

  rbd::ForwardDynamics fd(mb_);

  // compute C = cor/cent + G
  fd.computeC(mb_, mbc_);
  Eigen::VectorXd N = fd.C();

  // compute G
  Eigen::VectorXd G(Dof());
  GravityVector(G);

  // solve for C
  C = N - G;
}

void RBDyn::CoriolisMatrix(Eigen::MatrixXd& C) {
  rbd::Coriolis cor(mb_);
  C = cor.coriolis(mb_, mbc_);
}

int RBDyn::BodyIdFromName(const std::string& body_name) const {
  auto bd = mb_.bodies();

  for (int i = 0; i < bd.size(); i++) {
    if (bd[i].name() == body_name) {
      return i;
    }
  }

  return -1;
}

void RBDyn::SetGravity(const Eigen::Vector3d& vec) {
  // the gravity in the opposite direction when seen as acceleration of the
  // robot base, see RNEA algorithms for clarification
  mbc_.gravity = -vec;
}

}  // namespace rigid_body_dynamics
}  // namespace robotics
