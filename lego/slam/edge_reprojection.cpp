#include "lego/slam/edge_reprojection.h"
#include "lego/slam/vertex_pose.h"
#include "lego/base/utility.h"
#include "lego/thirdparty/Sophus/sophus/se3.hpp"

namespace lego {

    void EdgeReprojection::computeResidual() {
        double inv_dep_i = vertexes_[0]->getEstimate()(0, 0);

        VecX param_i = vertexes_[1]->getEstimate();
        Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
        Vec3 Pi = param_i.head<3>();

        VecX param_j = vertexes_[2]->getEstimate();
        Qd Qj(param_j[6], param_j[3], param_j[4], param_j[5]);
        Vec3 Pj = param_j.head<3>();

        VecX param_ext = vertexes_[3]->getEstimate();
        Qd qic(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
        Vec3 tic = param_ext.head<3>();

        Vec3 pts_camera_i = pts_i_ / inv_dep_i;
        Vec3 pts_imu_i = qic * pts_camera_i + tic;
        Vec3 pts_w = Qi * pts_imu_i + Pi;
        Vec3 pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Vec3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);

        double dep_j = pts_camera_j.z();
        /// J^t * J * delta_x = -J^t * r
        residual_ = (pts_camera_j / dep_j).head<2>() - measurement_;
    }

    //void EdgeReprojection::setTransformationIMUFromCamera(Eigen::Quaterniond &qic, Vec3 &tic) {
    //    qic_ = qic;
    //    tic_ = tic;
    //}

    void EdgeReprojection::computeJacobians() {
        double inv_dep_i = vertexes_[0]->getEstimate()(0, 0);

        VecX param_i = vertexes_[1]->getEstimate();
        Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
        Vec3 Pi = param_i.head<3>();

        VecX param_j = vertexes_[2]->getEstimate();
        Qd Qj(param_j[6], param_j[3], param_j[4], param_j[5]);
        Vec3 Pj = param_j.head<3>();

        VecX param_ext = vertexes_[3]->getEstimate();
        Qd qic(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
        Vec3 tic = param_ext.head<3>();

        Vec3 pts_camera_i = pts_i_ / inv_dep_i;
        Vec3 pts_imu_i = qic * pts_camera_i + tic;
        Vec3 pts_w = Qi * pts_imu_i + Pi;
        Vec3 pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Vec3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);

        double dep_j = pts_camera_j.z();

        Mat33 Ri = Qi.toRotationMatrix();
        Mat33 Rj = Qj.toRotationMatrix();
        Mat33 ric = qic.toRotationMatrix();
        Mat23 reduce(2, 3);
        reduce << 1.0 / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
                0, 1.0 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        Eigen::Matrix<double, 2, 6> jacobian_pose_i;
        Eigen::Matrix<double, 3, 6> jaco_i;
        jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
        jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Sophus::SO3d::hat(pts_imu_i);
        jacobian_pose_i.leftCols<6>() = reduce * jaco_i;

        Eigen::Matrix<double, 2, 6> jacobian_pose_j;
        Eigen::Matrix<double, 3, 6> jaco_j;
        jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
        jaco_j.rightCols<3>() = ric.transpose() * Sophus::SO3d::hat(pts_imu_j);
        jacobian_pose_j.leftCols<6>() = reduce * jaco_j;

        Eigen::Vector2d jacobian_feature;
        jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);

        Eigen::Matrix<double, 2, 6> jacobian_ex_pose;
        Eigen::Matrix<double, 3, 6> jaco_ex;
        jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
        Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
        jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                 Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
        jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;

        jacobians_[0] = jacobian_feature;
        jacobians_[1] = jacobian_pose_i;
        jacobians_[2] = jacobian_pose_j;
        jacobians_[3] = jacobian_ex_pose;
    }

    void EdgeReprojectionXYZ::computeResidual() {
        Vec3 pts_w = vertexes_[0]->getEstimate();

        VecX param_i = vertexes_[1]->getEstimate();
        Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
        Vec3 Pi = param_i.head<3>();

        Vec3 pts_imu_i = Qi.inverse() * (pts_w - Pi);
        Vec3 pts_camera_i = qic_.inverse() * (pts_imu_i - tic_);

        double dep_i = pts_camera_i.z();
        residual_ = (pts_camera_i / dep_i).head<2>() - measurement_;
    }

    void EdgeReprojectionXYZ::setTransformationIMUFromCamera(Eigen::Quaterniond &qic, Vec3 &tic) {
        qic_ = qic;
        tic_ = tic;
    }

    void EdgeReprojectionXYZ::computeJacobians() {
        Vec3 pts_w = vertexes_[0]->getEstimate();

        VecX param_i = vertexes_[1]->getEstimate();
        Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
        Vec3 Pi = param_i.head<3>();

        Vec3 pts_imu_i = Qi.inverse() * (pts_w - Pi);
        Vec3 pts_camera_i = qic_.inverse() * (pts_imu_i - tic_);

        double dep_i = pts_camera_i.z();

        Mat33 Ri = Qi.toRotationMatrix();
        Mat33 ric = qic_.toRotationMatrix();
        Mat23 reduce(2, 3);
        reduce << 1.0 / dep_i, 0, -pts_camera_i(0) / (dep_i * dep_i),
                0, 1.0 / dep_i, -pts_camera_i(1) / (dep_i * dep_i);

        Eigen::Matrix<double, 2, 6> jacobian_pose_i;
        Eigen::Matrix<double, 3, 6> jaco_i;
        jaco_i.leftCols<3>() = ric.transpose() * -Ri.transpose();
        jaco_i.rightCols<3>() = ric.transpose() * Sophus::SO3d::hat(pts_imu_i);
        jacobian_pose_i.leftCols<6>() = reduce * jaco_i;

        Eigen::Matrix<double, 2, 3> jacobian_feature;
        jacobian_feature = reduce * ric.transpose() * Ri.transpose();

        jacobians_[0] = jacobian_feature;
        jacobians_[1] = jacobian_pose_i;
    }

    void EdgeReprojectionPoseOnly::computeResidual() {
        VecX pose_params = vertexes_[0]->getEstimate();
        Sophus::SE3d pose(Qd(pose_params[6], pose_params[3], pose_params[4], pose_params[5]),
                          pose_params.head<3>());

        Vec3 pc = pose * landmark_world_;
        pc = pc / pc[2];
        Vec2 residual_pixel = (K_ * pc).head<2>() - measurement_;

        residual_ = residual_pixel;
    }

    void EdgeReprojectionPoseOnly::computeJacobians() {
        /// TODO
    }
}
