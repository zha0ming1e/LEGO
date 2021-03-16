/*
 * example: pose graph optimization with SE3
 */

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

/// lego headers
#include "lego/base/base_vertex.h"
#include "lego/base/base_edge.h"
#include "lego/base/cost_function.h"
#include "lego/base/problem.h"

#include "lego/thirdparty/Sophus/sophus/se3.hpp"

using namespace std;
using namespace Eigen;
using Sophus::SE3d;
using Sophus::SO3d;

/// the approximate of the inverse of right Jacobian on se(3)
Mat66 JRInv(const SE3d &e) {
    Mat66 J;
    Mat33 phi_hat = SO3d::hat(e.so3().log());
    J.block(0, 0, 3, 3) = phi_hat;
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Mat33::Zero(3, 3);
    J.block(3, 3, 3, 3) = phi_hat;

    J = J * 0.5 + Mat66::Identity();

    return J;
}

/// pose vertex on SE3
class VertexPose : public lego::BaseVertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPose() : lego::BaseVertex(4, 4, 6) {
        estimate_ = Mat44::Zero();
    }

    bool read(istream &is) {
        double data[7];
        for (double &i : data) is >> i;
        estimate_ = SE3d(Quaterniond(data[6], data[3], data[4], data[5]),
                         Vec3(data[0], data[1], data[2])).matrix();
    }

    bool write(ostream &os) const {
        os << getId() << " ";
        SE3d est = SE3d(estimate_);
        Quaterniond q = est.unit_quaternion();
        os << est.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;

        return true;
    }

    /// left multiplication
    /// overriding add
    void add(const VecX &update) override {
        Vec6 upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];

        estimate_ = (SE3d::exp(upd) * SE3d(estimate_)).matrix();
    }

    std::string getInfo() const override { return std::string("VertexPose"); }
};

/// edge between SE3
class EdgeSE3 : public lego::BaseEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3() : lego::BaseEdge(6, 2, 4, 4,
                               std::vector<std::string>{"VertexPose", "VertexPose"}) {
        residual_ = Vec6::Zero();
        measurement_ = Mat44::Zero();
    }

    bool read(istream &is) {
        double data[7];
        for (double &i : data) is >> i;
        measurement_ = SE3d(Quaterniond(data[6], data[3], data[4], data[5]),
                            Vec3(data[0], data[1], data[2])).matrix();

        int rows = information_.rows(), cols = information_.cols();
        for (int i = 0; i < rows && is.good(); i++) {
            for (int j = i; j < cols && is.good(); j++) {
                double info;
                is >> info;
                information_(i, j) = info;
                if (i != j)
                    information_(j, i) = info;
            }
        }
        return true;
    }

    bool write(ostream &os) const {
        os << vertexes_[0]->getId() << " " << vertexes_[1]->getId() << " ";
        SE3d m = SE3d(measurement_);
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        /// information matrix
        int rows = information_.rows(), cols = information_.cols();
        for (int i = 0; i < rows; i++) {
            for (int j = i; j < cols; j++) {
                os << information_(i, j) << " ";
            }
        }
        os << endl;
        return true;
    }

    /// residual
    void computeResidual() override {
        SE3d v1 = SE3d(vertexes_[0]->getEstimate());
        SE3d v2 = SE3d(vertexes_[1]->getEstimate());
        SE3d meas = SE3d(measurement_);
        residual_ = (meas.inverse() * v1.inverse() * v2).log();
    }

    /// jacobian
    void computeJacobians() override {
        //SE3d v1 = SE3d(vertexes_[0]->getEstimate());
        SE3d v2 = SE3d(vertexes_[1]->getEstimate());
        Mat66 J = JRInv(SE3d::exp(residual_));

        //jacobians_[0] = Mat66::Zero();
        jacobians_[0] = -J * v2.inverse().Adj();
        //jacobians_[1] = Mat66::Zero();
        jacobians_[1] = -1.0 * jacobians_[0];
    }

    std::string getInfo() const override { return std::string("EdgeSE3"); }
};

/// main
int main(int argc, char **argv) {
    if (argc != 3) {
        cout << "Usage: [RUN_FILE_example_pose_graph] [sphere.g2o] [STRATEGY_NO.] \nError: argc is not 3. " << endl;
        return 1;
    }

    ifstream fin(argv[1]);
    if (!fin) {
        cout << "Can not open file: " << argv[1] << ". \nPlease check the file path. " << endl;
        return 2;
    }

    /// build the problem and initial settings
    /// default LM algorithm and strategy
    lego::Problem problem(lego::Problem::ProblemType::SLAM);
    std::string strategy_no;
    if (std::stoi(argv[2]) == 0) {
        //problem.setStrategyType(lego::Problem::StrategyType::DEFAULT);
        strategy_no = "0";
    } else if (std::stoi(argv[2]) == 1) {
        /// default LM algorithm and strategy 1
        problem.setStrategyType(lego::Problem::StrategyType::STRATEGY1);
        strategy_no = "1";
    } else {
        std::cerr << "Usage: [RUN_FILE_example_pose_graph] [sphere.g2o] [STRATEGY_NO.] "
                  << "\nError: strategy index: " << argv[2] << " dose not exist. " << std::endl;
        return 2;
    }

    int vertexCnt = 0, edgeCnt = 0;

    vector<std::shared_ptr<VertexPose>> vectexes;
    vector<std::shared_ptr<EdgeSE3>> edges;
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // vertex
            std::shared_ptr<VertexPose> v(new VertexPose());
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            problem.addVertex(v);
            vertexCnt++;
            vectexes.push_back(v);
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // edge
            std::shared_ptr<EdgeSE3> e(new EdgeSE3());
            int idx1, idx2;
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);

            std::vector<std::shared_ptr<lego::BaseVertex>> e_v;
            e_v.push_back(problem.getAllVertexes()[idx1]);
            e_v.push_back(problem.getAllVertexes()[idx2]);
            e->setVertex(e_v);
            e->read(fin);
            problem.addEdge(e);
            edges.push_back(e);
        }
        if (!fin.good()) break;
    }

    cout << "Example: Pose Graph Optimization with SE3 start... " << endl;
    cout << "Read Total: " << "\nVertexCount = " << vertexCnt << ", EdgeCount = " << edgeCnt << endl;

    cout << "Optimizing..." << endl;
    //problem.setInitialLambda(1000);
    problem.solve(50);

    cout << "\nSaving optimization results..." << endl;

    // output with g2o format
    ofstream fout("result_SE3_" + strategy_no + ".g2o");
    for (auto &v : vectexes) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for (auto &e : edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();

    return 0;
}
