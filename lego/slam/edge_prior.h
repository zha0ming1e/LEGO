#ifndef LEGO_EDGEPRIOR_H
#define LEGO_EDGEPRIOR_H

#include "lego/base/eigen3_types.h"
#include "lego/base/base_edge.h"
#include "lego/thirdparty/Sophus/sophus/se3.hpp"

namespace lego {

    /*
     * EdgeSE3Prior
     *
     * just 1 pose vertex connected to it: Ti
     */
    class EdgeSE3Prior : public BaseEdge {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeSE3Prior(const Vec3 &p, const Qd &q) :
                BaseEdge(6, 1, 6, 1,
                         std::vector<std::string>{"VertexPose"}) {
            Qp_ = q;
            Pp_ = p;

            //Sophus::SO3d rq(q);
            /// rotation prior
            //measurement_.block<3, 1>(0, 0) = Sophus::SO3d::log(rq);
            /// translation prior
            //measurement_.block<3, 1>(3, 0) = p;
        }

        std::string getInfo() const override { return "EdgeSE3Prior"; }

        void computeResidual() override;

        void computeJacobians() override;

    private:
        /// rotation prior
        Qd Qp_;
        /// translation prior
        Vec3 Pp_;
    };
}
#endif  // LEGO_EDGEPRIOR_H
