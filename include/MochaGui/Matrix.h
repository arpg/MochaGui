#pragma once

#include <Eigen/Eigen>
#include <Sophus/se3.hpp>
#include "Messages.pb.h"

namespace ninjacar {

/// Taken from HAL/Messages/Matrix.h
inline void ReadMatrix(const MatrixMsg &msg, Eigen::MatrixXd* mat) {
  mat->resize(msg.rows(),msg.data_size()/msg.rows());
  for(int ii = 0 ; ii < msg.data_size() ; ii++){
    mat->operator()(ii) = msg.data(ii);
  }
}

inline void ReadVector(const VectorMsg &msg, Eigen::VectorXd* vec) {
  vec->resize(msg.data_size());
  for(int ii = 0 ; ii < msg.data_size() ; ii++){
    vec->operator()(ii) = msg.data(ii);
  }
}

inline void WriteMatrix(const Eigen::MatrixXd &mat, MatrixMsg *msg) {
  msg->set_rows(mat.rows());
  msg->mutable_data()->Reserve(mat.rows()*mat.cols());
  for(int ii = 0 ; ii < mat.cols()*mat.rows() ; ii++){
    msg->add_data(mat(ii));
  }
}

inline void WriteVector(const Eigen::VectorXd &mat, VectorMsg *msg) {
  msg->mutable_data()->Reserve(mat.rows());
  for(int ii = 0 ; ii < mat.rows() ; ii++){
    msg->add_data(mat(ii));
  }
}

/// Taken from HAL/Messages/Pose.h
template <typename Scalar>
void WritePoseSE3(const Sophus::SE3Group<Scalar>& pose, PoseMsg* msg) {
  msg->set_type(PoseMsg_Type_SE3);

  Eigen::Matrix<Scalar, 7, 1> vec(
      Sophus::Map<const Eigen::Matrix<Scalar, 7, 1> >(pose.data()));
  WriteVector(vec.template cast<double>(), msg->mutable_pose());
}

template <typename Scalar>
void ReadPoseSE3(const PoseMsg &msg, Sophus::SE3Group<Scalar>* pose) {
  *pose = Sophus::SE3d(Eigen::Map<const Sophus::SE3d>(
      msg.pose().data().data())).template cast<Scalar>();
}

} // namespace ninjacar
