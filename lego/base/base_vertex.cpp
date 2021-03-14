#include "base_vertex.h"

namespace lego {

    unsigned long global_vertex_id = 0;

    BaseVertex::BaseVertex(int dim, int dof) {
        estimate_.resize(dim, 1);
        dof_ = dof > 0 ? dof : dim;
        id_ = global_vertex_id++;
    }

    BaseVertex::~BaseVertex() = default;

    int BaseVertex::getDim() const {
        return estimate_.rows();
    }

    int BaseVertex::getDoF() const {
        return dof_;
    }

    void BaseVertex::add(const VecX &delta) {
        /// linear vector addition for default
        estimate_ += delta;
    }
}
