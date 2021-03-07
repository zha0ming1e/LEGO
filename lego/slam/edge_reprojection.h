#ifndef LEGO_EDGEREPROJECTION_H
#define LEGO_EDGEREPROJECTION_H

#include "lego/base/eigen3_types.h"
#include "lego/base/base_edge.h"

namespace lego {

    /*
     * EdgeReprojection
     *
     * 4 vertexes connected to it:
     * inverse depth,
     * T_World_From_Body1 of source camera，
     * T_World_From_Body2 of measurement camera
     * T_IMU_Camera extrinsics
     *
     * NOTE: the order of vertexes
     */
    class EdgeReprojection : public BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeReprojection(const Vec3 &pts_i, const Vec3 &pts_j)
                : BaseEdge(2, 4,
                           std::vector<std::string>{"VertexInverseDepth", "VertexPose", "VertexPose", "VertexPose"}) {
            pts_i_ = pts_i;
            /// set measurement
            measurement_ = pts_j.head<2>();
        }

        std::string getInfo() const override { return "EdgeReprojection"; }

        void computeResidual() override;

        void computeJacobians() override;

        //void setTransformationIMUFromCamera(Eigen::Quaterniond &qic, Vec3 &tic);

    private:
        /// transformation imu from camera
        //Qd qic_;
        //Vec3 tic_;

        /// coordinate of point i
        Vec3 pts_i_;
    };

    /*
     * EdgeReprojectionXYZ
     *
     * 2 vertexes connected to it:
     * point coordinate: xyz,
     * T_World_From_Body of camera
     *
     * NOTE: the order of vertexes
     */
    class EdgeReprojectionXYZ : public BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        explicit EdgeReprojectionXYZ(const Vec3 &pts_i)
                : BaseEdge(2, 2, std::vector<std::string>{"VertexXYZ", "VertexPose"}) {
            /// set measurement
            measurement_ = pts_i.head<2>();
        }

        std::string getInfo() const override { return "EdgeReprojectionXYZ"; }

        void computeResidual() override;

        void computeJacobians() override;

        void setTransformationIMUFromCamera(Eigen::Quaterniond &qic, Vec3 &tic);

    private:
        /// transformation imu from camera
        Qd qic_;
        Vec3 tic_;
    };

    /*
     * EdgeReprojectionPoseOnly
     *
     * just 1 pose vertex connected to it
     */
    class EdgeReprojectionPoseOnly : public BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeReprojectionPoseOnly(const Vec3 &landmark_world, const Mat33 &K) :
                BaseEdge(2, 1, std::vector<std::string>{"VertexPose"}),
                landmark_world_(landmark_world), K_(K) {}

        std::string getInfo() const override { return "EdgeReprojectionPoseOnly"; }

        void computeResidual() override;

        void computeJacobians() override;

    private:
        Vec3 landmark_world_;
        Mat33 K_;
    };
}
#endif  // LEGO_EDGEREPROJECTION_H
