/*
 * example: non-linear fitting
 * fitting model: y = exp(a*x^2 + b*x + c)
 */

#include <random>

/// lego headers
#include "lego/base/problem.h"

/// data fitting vertex
class DataFittingVertex : public lego::BaseVertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// 3 dim
    DataFittingVertex() : BaseVertex(3) {}
    std::string getInfo() const override { return "DataFittingVertex"; }
};

/// data fitting edge
class DataFittingEdge : public lego::BaseEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit DataFittingEdge(double x) : BaseEdge(1,1, std::vector<std::string>{"a, b, c"}), x_(x) {}
    /// compute residual
    void computeResidual() override {
        /// estimate
        Vec3 abc = vertexes_[0]->getEstimate();
        /// residual: convert estimate to VecX
        residual_ = Vec1(std::exp(abc(0)*x_*x_ + abc(1)*x_ + abc(2))) - measurement_;
    }
    /// compute jacobians
    void computeJacobians() override {
        Vec3 abc = vertexes_[0]->getEstimate();
        double exp_y = std::exp(abc(0)*x_*x_ + abc(1)*x_ + abc(2));

        /// jacobian is 1 X 3
        Eigen::Matrix<double, 1, 3> jaco_abc;
        jaco_abc << x_*x_*exp_y, x_*exp_y , 1*exp_y;
        jacobians_[0] = jaco_abc;
    }
    std::string getInfo() const override { return "DataFittingEdge"; }

public:
    double x_;
};

//////////////////////// main ////////////////////////
int main(int argc, char **argv) {
    /// ground truth
    double a = 1.0, b = 2.0, c = 1.0;
    int N = 1000;
    /// noise sigma
    double w_sigma= 1.0;

    std::default_random_engine generator;
    std::normal_distribution<double> noise(0, w_sigma);

    /// build the problem
    lego::Problem problem(lego::Problem::ProblemType::BASE);

    /// vertex
    std::shared_ptr<DataFittingVertex> vertex(new DataFittingVertex());
    /// set estimate
    vertex->setEstimate(Eigen::Vector3d (0,0,0));
    problem.addVertex(vertex);

    /// measurements
    for (int i = 0; i < N; ++i) {
        double x = ((double) i) / N;
        double n = noise(generator);

        /// data with noise
        double y = std::exp(a*x*x + b*x + c) + n;

        /// edge
        std::shared_ptr<DataFittingEdge> edge(new DataFittingEdge(x));
        /// set measurement
        edge->setMeasurement(Vec1(y));

        std::vector<std::shared_ptr<lego::BaseVertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->setVertex(edge_vertex);

        problem.addEdge(edge);
    }

    std::cout << "\nExample: Non-linear Fitting start... " << std::endl;
    /// LM
    //problem.setVerbose(false);
    problem.solve(30);

    auto esti = vertex->getEstimate();
    std::cout << "\n--------Estimates after optimization--------" << std::endl;
    std::cout << "a, b, c = " << esti(0) << ", " << esti(1) << ", " << esti(2) << std::endl;
    std::cout << "--------Ground truth--------" << std::endl;
    std::cout << "a, b, c = 1.0, 2.0, 1.0" << std::endl;

    return 0;
}
