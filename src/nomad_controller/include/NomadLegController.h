/*
 * NomadLegController.h
 *
 *  Created on: June 24, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef NOMAD_LEGCONTROLLER_H_
#define NOMAD_LEGCONTROLLER_H_

// C System Files

// C++ System Files

// Third Party Includes
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/simulation/World.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

#include <Eigen/Dense>

// Project Include Files

// 3 dof leg controller
class NomadLegController
{
public:
double max_tau = 0.0;
// TODO: Offset to actual contact point
// TODO: Assumes sane indexing.  May need to support more explicit ids in constructor
   NomadLegController(dart::dynamics::SkeletonPtr &parent, int haa_dof_id, int haa_bn_id, int foot_bn_id) : parent_skeleton_(parent)
   {
     
     haa_body_ = parent_skeleton_->getBodyNode(haa_bn_id);
     foot_body_ = parent_skeleton_->getBodyNode(foot_bn_id);

     DOF_[0] = parent_skeleton_->getDof(haa_dof_id);
     DOF_[1] = parent_skeleton_->getDof(haa_dof_id + 1);
     DOF_[2] = parent_skeleton_->getDof(haa_dof_id + 2);

     Reset();
   }

  void Reset()
  {
    k_P_cartesian_ = Eigen::Matrix3d::Zero();
    k_D_cartesian_ = Eigen::Matrix3d::Zero();

    k_P_joint_ = Eigen::Matrix3d::Zero();
    k_D_joint_ = Eigen::Matrix3d::Zero();

    foot_pos_desired_ = Eigen::Vector3d::Zero();
    foot_vel_desired_ = Eigen::Vector3d::Zero();

    q_desired_ = Eigen::Vector3d::Zero();
    qd_desired_ = Eigen::Vector3d::Zero();
    tau_feedforward_ = Eigen::Vector3d::Zero();

    force_feedforward_ = Eigen::Vector3d::Zero();

  }

  void SetCartesianPD(const Eigen::Vector3d &k_P, const Eigen::Vector3d &k_D)
  {
    k_P_cartesian_ = k_P.asDiagonal();
    k_D_cartesian_ = k_D.asDiagonal();
  }

  void SetJointPD(const Eigen::Vector3d &k_P, const Eigen::Vector3d &k_D)
  {
    k_P_joint_ = k_P.asDiagonal();
    k_D_joint_ = k_D.asDiagonal();
  }

  void SetFootStateDesired(const Eigen::Vector3d &posDesired, const Eigen::Vector3d &velDesired)
  {
    foot_pos_desired_ = posDesired;
    foot_vel_desired_ = velDesired;
  }
  void SetForceFeedForward(const Eigen::Vector3d &force)
  {
    force_feedforward_ = force;
  }
  const Eigen::Vector3d &GetFootPosition()
  {
    return foot_pos_;
  }
  void Run()
  {
    // Compute Feed Forwards
    Eigen::Vector3d tau_output;
    Eigen::Vector3d force_output;

     tau_output = tau_feedforward_;
     force_output = force_feedforward_;
     force_output += k_P_cartesian_ * (foot_pos_desired_ - foot_pos_);
     force_output += k_D_cartesian_ * (foot_vel_desired_ - foot_vel_);

     //std::cout << "Force Output: Z" << (force_output)[2] << std::endl;
     // std::cout << leg_skeleton_->getGravityForces() << std::endl;

     // Convert to Joint Torques
     //std::cout << J_.rows() << " to: " << J_.cols() << std::endl;
     Eigen::VectorXd output = (J_.transpose() * force_output);
     Eigen::VectorXd tau_temp = output.segment(DOF_[0]->getIndexInSkeleton(), 3);


     //std::cout << output << std::endl;
     //std::cout << output.rows() << " to: " << output.cols() << std::endl;
     //tau_output += J_.transpose() * force_output;
    tau_output += tau_temp;

  // if(DOF_[0]->getIndexInSkeleton() == 6)
  // {
  //   std::cout << tau_output[0] << ", " << tau_output[1] << ", " << tau_output[2] << std::endl;
  //   max_tau = std::max(std::abs(tau_output[2]), max_tau);

  //   std::cout << max_tau << std::endl;
  // }
     //std::cout << tau_output << std::endl;
     //max_torque = std::max(max_torque, tau_output[2]);
     //std::cout << max_torque << std::endl;
     // Technically sets a force on the unactuated joint = 0.  Just be aware. I think we are just lucky it happens
     // to match 3dof even though this leg setup is 2dof
     //parent_skeleton_->setForce(DOF_[0])

     DOF_[0]->setForce(tau_output[0]);
     DOF_[1]->setForce(tau_output[1]);
     DOF_[2]->setForce(tau_output[2]);
    //parent_skeleton_->setForces(tau_output);
  }

  void UpdateState()
  {
    // Compute Jacobian (Linear)  TODO: Do we want the full Jacobian with angular velocities?
    J_ = parent_skeleton_->getLinearJacobian(foot_body_, haa_body_);
    //std::cout << foot_body_->getName() << " J: " << J_ << std::endl;
    //std::cout << DOF_[0]->getIndexInSkeleton() << std::endl;
    foot_pos_ = foot_body_->getTransform(haa_body_).translation();
    foot_vel_ = J_ * parent_skeleton_->getVelocities();

    //std::cout << J_.rows();

    // This is the same, but like using the jacobian better as it is more generic
    // TODO: See if linearvelocity is faster/cached for some reason
    //foot_vel_ = foot_body_->getLinearVelocity(hfe_body_, hfe_body_);
  }

   const dart::dynamics::SkeletonPtr &Skeleton()
   {
     return parent_skeleton_;
   }

protected:
  dart::dynamics::SkeletonPtr parent_skeleton_;
  dart::dynamics::BodyNodePtr haa_body_;
  dart::dynamics::BodyNodePtr foot_body_;
  dart::dynamics::DegreeOfFreedomPtr DOF_[3]; // 0 = haa, 1 = hfe, 2 = kfe

  Eigen::Matrix3d k_P_joint_;
  Eigen::Matrix3d k_D_joint_;
  Eigen::Matrix3d k_P_cartesian_;
  Eigen::Matrix3d k_D_cartesian_;

  Eigen::Vector3d foot_pos_desired_;
  Eigen::Vector3d foot_vel_desired_;

  Eigen::Vector3d foot_pos_;
  Eigen::Vector3d foot_vel_;

  Eigen::Vector3d q_desired_;  // 3Dof Leg
  Eigen::Vector3d qd_desired_; // 3Dof Leg

  Eigen::Vector3d q_;
  Eigen::Vector3d qd_;

  Eigen::Vector3d force_feedforward_;
  Eigen::Vector3d tau_feedforward_;

  Eigen::MatrixXd J_; // Leg Jacobian

};

#endif // NOMAD_LEGCONTROLLER_H_

  