#ifndef LEGO_VERTEXPOINTXYZ_H
#define LEGO_VERTEXPOINTXYZ_H

#include "lego/base/eigen3_types.h"
#include "lego/base/base_vertex.h"

namespace lego {

    /*
     * VertexPointXYZ
     *
     * 3D point landmark
     */
    class VertexPointXYZ : public BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPointXYZ() : BaseVertex(3) {}

        std::string getInfo() const override { return "VertexPointXYZ"; }
    };
}
#endif  // LEGO_VERTEXPOINTXYZ_H
