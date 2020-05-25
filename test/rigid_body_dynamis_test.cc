#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include <robotics/rigid_body_dynamics/rigid_body_dynamics.h>

namespace robotics {
namespace rigid_body_dynamics {
// Test from google test primer
// https://github.com/google/googletest/blob/master/googletest/docs/primer.md

bool MatrixEquality(const Eigen::MatrixXd &left, const Eigen::MatrixXd &right) {
  return left.isApprox(right, 1e-4);
}

class RBDynTest : public ::testing::Test {
 protected:
  RBDynTest() {
    // setup tests
    rb_.Dof();
    rb_.CreateModel("../resource/FrankaEmikaSubChain.urdf");
    rb_.SetGravity(Eigen::Vector3d(0, 0, -9.81));

    Eigen::VectorXd q(rb_.Dof());
    Eigen::VectorXd dq(rb_.Dof());

    q << 1, 2, 3, 4;
    dq << 5, 6, 7, 8;

    rb_.SetQ(q);
    rb_.SetDq(dq);
    rb_.ForwardKinematics();

    // configurations
    R_ << 0.5403023, -0.841471, 0, 0.841471, 0.5403023, 0, 0, 0, 1;
    p_ << 1, 2, 3;
  }
  virtual ~RBDynTest() {}

  // SetUp test will be called right after constructor and right before its
  // tests if we need further setup of the test
  virtual void SetUp() {}

  // TearDown test that will be called right after each test and right before
  // its destructor to clean up additional objects created by SetUp
  virtual void TearDown() {}

  // Test subject
  RBDyn rb_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d p_;
  std::string body_name_{"panda_link4"};
};

// test dof
TEST_F(RBDynTest, TestDof) { EXPECT_EQ(rb_.Dof(), 4); }

// test pose
TEST_F(RBDynTest, TestEEPose) {
  Eigen::MatrixXd Tref(4, 4);
  Eigen::MatrixXd T(4, 4);

  Tref << -0.4397, -0.2425, -0.8648, 0.1638, -0.8555, -0.1801, 0.4855, 0.2767,
      -0.2735, 0.9533, -0.1283, 0.2758, 0, 0, 0, 1.0000;
  rb_.EndEffPose(T, body_name_);

  ASSERT_PRED2(MatrixEquality, Tref, T);
}

// test pose with offset
TEST_F(RBDynTest, TestEEPoseOfs) {
  Eigen::MatrixXd Tref(4, 4);
  Eigen::MatrixXd T(4, 4);

  Tref << -0.4417, 0.2389, -0.8648, -3.3553, -0.6138, 0.6226, 0.4855, 0.5175,
      0.6544, 0.7452, -0.1283, 1.5239, 0, 0, 0, 1.0000;
  rb_.EndEffPose(T, body_name_, R_, p_);

  ASSERT_PRED2(MatrixEquality, Tref, T);
}

// test jacobian
TEST_F(RBDynTest, TestJ) {
  Eigen::MatrixXd Jref(6, rb_.Dof());
  Eigen::MatrixXd J(6, rb_.Dof());

  rb_.Jacobian(J, body_name_);

  Jref << -0.2767, -0.0309, 0.0713, 0, 0.1638, -0.0482, -0.0401, 0, 0, -0.3213,
      0.0106, 0, 0, -0.8415, 0.4913, -0.8648, 0, 0.5403, 0.7651, 0.4855, 1.0000,
      0, -0.4161, -0.1283;

  ASSERT_PRED2(MatrixEquality, Jref, J);
}

// test jacobian with offset
TEST_F(RBDynTest, TestJOfs) {
  Eigen::MatrixXd Jref(6, rb_.Dof());
  Eigen::MatrixXd J(6, rb_.Dof());

  rb_.Jacobian(J, body_name_, p_);

  Jref << -0.5175, 0.6434, 1.1266, 0.6368, -3.3553, 1.0021, 0.8112, 1.5309, 0,
      1.3774, 2.8215, 1.5002, 0, -0.8415, 0.4913, -0.8648, 0, 0.5403, 0.7651,
      0.4855, 1.0000, 0, -0.4161, -0.1283;

  ASSERT_PRED2(MatrixEquality, Jref, J);
}

// test jacobian dot
TEST_F(RBDynTest, TestDJ) {
  Eigen::MatrixXd dJref(6, rb_.Dof());
  Eigen::MatrixXd dJ(6, rb_.Dof());

  rb_.JacobianDot(dJ, body_name_);

  dJref << -0.2497, -0.7608, 0.1746, 0, -1.0695, -1.7146, 0.1659, 0, 0, 0.3095,
      -0.5489, 0, 0, -2.7015, -5.1748, -2.1165, 0, -4.2074, 0.3554, -2.0113, 0,
      0, -5.4558, 6.6537;
  ASSERT_PRED2(MatrixEquality, dJref, dJ);
}

// test jacobian dot with offset
TEST_F(RBDynTest, TestDJOfs) {
  Eigen::MatrixXd dJref(6, rb_.Dof());
  Eigen::MatrixXd dJ(6, rb_.Dof());

  rb_.JacobianDot(dJ, body_name_, p_);

  dJref << -7.1622, 16.6108, 36.8460, 17.1018, 14.2540, 36.8904, -1.1235,
      13.4692, 0, -29.2431, -8.8729, -21.0047, 0, -2.7015, -5.1748, -2.1165, 0,
      -4.2074, 0.3554, -2.0113, 0, 0, -5.4558, 6.6537;

  ASSERT_PRED2(MatrixEquality, dJref, dJ);
}

// test M
TEST_F(RBDynTest, TestM) {
  Eigen::MatrixXd M(rb_.Dof(), rb_.Dof());
  Eigen::MatrixXd Mref(rb_.Dof(), rb_.Dof());

  rb_.MassMatrix(M);

  Mref << 0.5305142, 0.0019764, -0.152168, -0.0040797, 0.0019764, 0.532483,
      -0.019742, 0.0292846, -0.152168, -0.019742, 0.0709032, -0.0001147,
      -0.0040797, 0.0292846, -0.0001147, 0.016044;

  ASSERT_PRED2(MatrixEquality, Mref, M);
}

// test C
TEST_F(RBDynTest, TestC) {
  Eigen::VectorXd C(rb_.Dof());
  Eigen::VectorXd Cref(rb_.Dof());

  rb_.CoriolisVector(C);

  Cref << -1.3034, 14.54867, -2.42903, -0.3757;

  ASSERT_PRED2(MatrixEquality, Cref, C);
}

// test G
TEST_F(RBDynTest, TestG) {
  Eigen::VectorXd G(rb_.Dof());
  Eigen::VectorXd Gref(rb_.Dof());

  rb_.GravityVector(G);

  Gref << 0, -16.25, 0.54005, -0.587171;

  ASSERT_PRED2(MatrixEquality, Gref, G);
}

// test Cm
TEST_F(RBDynTest, TestCm) {
  Eigen::VectorXd Cref(rb_.Dof());
  Eigen::MatrixXd Cm(rb_.Dof(), rb_.Dof());

  rb_.CoriolisVector(Cref);
  rb_.CoriolisMatrix(Cm);

  Eigen::VectorXd dq(rb_.Dof());
  rb_.Dq(dq);

  ASSERT_PRED2(MatrixEquality, Cref, Cm * dq);
}

}  // namespace rigid_body_dynamics
}  // namespace robotics
