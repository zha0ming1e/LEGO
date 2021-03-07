#ifndef LEGO_VERTEXSPEEDBIAS_H
#define LEGO_VERTEXSPEEDBIAS_H

#include "lego/base/eigen3_types.h"
#include "lego/base/base_vertex.h"

namespace lego {

    /*
     * VertexSpeedBias
     *
     * estimate: v, ba, bg -> 9 DoF
     */
    class VertexSpeedBias : public BaseVertex {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexSpeedBias() : BaseVertex(9) {}

        std::string getInfo() const override { return "VertexSpeedBias"; }
    };
}
#endif
