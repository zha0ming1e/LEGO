#ifndef LEGO_VERTEXPOSE_H
#define LEGO_VERTEXPOSE_H

#include "lego/base/eigen3_types.h"
#include "lego/base/base_vertex.h"

namespace lego {

    /*
     * VertexPose
     *
     * estimate: tx, ty, tz, qx, qy, qz, qw -> 7 DoF
     * the optimization is perform on manifold, so update is 6 DoF, left multiplication
     * pose is represented as Twb in VIO case
     */
    class VertexPose : public BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPose() : BaseVertex(7, 1, 6) {}

        /// add override
        void add(const VecX &delta);

        std::string getInfo() const override { return "VertexPose"; }
    };
}
#endif  // LEGO_VERTEXPOSE_H
