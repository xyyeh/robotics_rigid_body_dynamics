/**
 * \class ExpClass
 *
 * \brief Provides an example classs
 *
 * This class is meant as an example.  It is not useful by itself
 * rather its usefulness is only a function of how much it helps
 * the reader.  It is in a sense defined by the person who reads it
 * and otherwise does not exist in any real form. Referenced from:
 * https://www-numi.fnal.gov/offline_software/srt_public_context/WebDocs/doxygen-howto.html
 *
 * \note Attempts at zen rarely work.
 * \author (last to touch it) $Author: bv $
 * \version $Revision: 1.5$
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: bv@bnl.gov
 * Created on: Wed Apr 13 18:39:37 2005
 */

#pragma once

#include <RBDyn/Body.h>
#include <RBDyn/Coriolis.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/Joint.h>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

namespace robotics {
namespace rigid_body_dynamics {

class RBDyn {
 private:
  /** multi body data */
  rbd::MultiBody mb_;
  rbd::MultiBodyConfig mbc_;
  rbd::MultiBodyGraph mbg_;

 public:
  RBDyn() = default;
  virtual ~RBDyn(){};

  /**
   * @brief Imports urdf and creates a multi-body model
   * @param[in] urdf_file_path Path to urdf file
   */
  void CreateModel(const std::string& urdf_file_path);

  /**
   * @brief Set link angles, q in [rad]
   * @param[in] q Link positions
   */
  void SetQ(const Eigen::VectorXd& q);

  /**
   * @brief Set link velocities, dq in [rad/s]
   * @param[in] dq Link velocities
   */
  void SetDq(const Eigen::VectorXd& dq);

  /**
   * @brief Get link angles, q in [rad]
   * @param[out] q Link angles
   */
  void Q(Eigen::VectorXd& q);

  /**
   * @brief Get link velocities, dq in [rad/s]
   * @param[out] dq Link velocities
   */
  void Dq(Eigen::VectorXd& dq);

  /**
   * @brief Retrieves number of DOF of the system
   * @return DOF of system
   */
  unsigned int Dof() const { return mb_.nrDof(); }

  /**
   * @brief Forward Kinematics to project joint space values to task space
   * values
   */
  void ForwardKinematics();

  /**
   * @brief Retrieves end effector pose given an offset transformation
   * @param[out] O_T_ee SE(3) transformation matrix of the end effector in world
   * frame
   * @param[in] body_name Name of the body i that the end-effector is located
   * @param[in] ofs_rotation Offset rotation from the body's frame i_R_ee
   * @param[in] ofs_position Offset position on the body i_p_ee
   */
  void EndEffPose(Eigen::MatrixXd& O_T_ee, const std::string& body_name,
                  const Eigen::Matrix3d& ofs_rotation,
                  const Eigen::VectorXd& ofs_position);

  /**
   * @brief Retrieves end effector pose given an offset transformation
   * @param[out] O_T_ee SE(3) transformation matrix of the end effector in world
   * frame
   * @param[in] body_name Name of the body i that the end-effector is located
   * @param[in] ofs_rotation Offset rotation from the body's frame i_R_ee
   * @param[in] ofs_position Offset position on the body i_p_ee
   */
  void EndEffPose(
      Eigen::MatrixXd& O_T_ee, const std::string& body_name,
      const Eigen::Matrix3d& ofs_rotation = Eigen::Matrix3d::Identity(3, 3),
      const Eigen::Vector3d& ofs_position = Eigen::Vector3d::Zero());

  /**
   * @brief Retrieves end effector jacobian given an offset transformation
   * @param[out] O_J_ee SE(3) Jacobian matrix of the end effector in world
   * frame
   * @param[in] body_name Name of the body i that the end-effector is located
   * @param[in] ofs_position Offset position on the body i_p_ee
   */
  void Jacobian(Eigen::MatrixXd& O_J_ee, const std::string& body_name,
                const Eigen::Vector3d& ofs_position = Eigen::Vector3d::Zero());

  /**
   * @brief Retrieves the time-derivative of end effector jacobian given an
   * offset transformation
   * @param[out] O_dJ_ee SE(3) Time-derivative of Jacobian matrix of the end
   * effector in world frame
   * @param[in] body_name Name of the body i that the end-effector is located
   * @param[in] ofs_position Offset position on the body i_p_ee
   */
  void JacobianDot(
      Eigen::MatrixXd& O_dJ_ee, const std::string& body_name,
      const Eigen::Vector3d& ofs_position = Eigen::Vector3d::Zero());

  /**
   * @brief Retrieves the mass matrix
   * @param[out] M Mass matrix of the system
   */
  void MassMatrix(Eigen::MatrixXd& M);

  /**
   * @brief Retrieves the gravity compensation vector
   * @param[out] G Gravity compensation vector of the system
   */
  void GravityVector(Eigen::VectorXd& G);

  /**
   * @brief Retrieves the coriolis/centrifugal vector
   * @param[out] C Coriolis vector of the system
   */
  void CoriolisVector(Eigen::VectorXd& C);

  /**
   * @brief Retrieves the coriolis/centrifugal matrix
   * @param[out] C Coriolis matrix of the system
   */
  void CoriolisMatrix(Eigen::MatrixXd& C);

  /**
   * @brief Retrieves the body id given the name
   * @param[in] body_name Name of the body
   * @return Id of the body, where id >= 0 indicates that the body is found
   */
  int BodyIdFromName(const std::string& body_name) const;

  /**
   * @brief Sets gravity for system
   * @param[in] vec Gravity vector
   */
  void SetGravity(const Eigen::Vector3d& vec);
};

}  // namespace rigid_body_dynamics
}  // namespace robotics