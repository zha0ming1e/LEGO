#ifndef LEGO_VERTEXINVERSEDEPTH_H
#define LEGO_VERTEXINVERSEDEPTH_H

#include "lego/base/eigen3_types.h"
#include "lego/base/base_vertex.h"

namespace lego {

    /*
     * VertexInverseDepth
     *
     * the inverse depth of a landmark
     */
    class VertexInverseDepth : public BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexInverseDepth() : BaseVertex(1) {}

        std::string getInfo() const override { return "VertexInverseDepth"; }
    };
}
#endif  // LEGO_VERTEXINVERSEDEPTH_H
