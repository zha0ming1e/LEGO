#include "vertex_pose.h"
#include "lego/thirdparty/Sophus/sophus/se3.hpp"

namespace lego {

    void VertexPose::add(const VecX &delta) {
        VecX esti = VecX(getEstimate());

        /// vector, quaternion, lie algebra additions
        esti.head<3>() += delta.head<3>();

        Qd q(esti[6], esti[3], esti[4], esti[5]);
        /// right multiplication with so3
        q = q * Sophus::SO3d::exp(Vec3(delta[3], delta[4], delta[5])).unit_quaternion();
        q.normalized();

        /// add
        esti[3] = q.x();
        esti[4] = q.y();
        esti[5] = q.z();
        esti[6] = q.w();
    }
}
