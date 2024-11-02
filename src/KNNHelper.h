//
// Created by lei on 11/11/2023.
//

#ifndef PARTICLESYSTEM_KNNHELPER_H
#define PARTICLESYSTEM_KNNHELPER_H

#include <omp.h>
#include <Eigen/Dense>
#include <nanoflann.hpp>

namespace psystem::knn_helper {
    using namespace Eigen;

    template<class KDTree>
    std::vector<Vector3d> getNeighborPoints(const KDTree &kdTree,
                                            const size_t &query_i,
                                            const MatrixXd &queryV,
                                            const size_t &num_results) {
        // do a knn search
        std::vector<typename KDTree::IndexType> ret_index(num_results + 1);
        std::vector<double> out_dist_sqr(num_results + 1);

        std::vector<double> query_pt;
        query_pt.emplace_back(queryV.row(query_i)(0));
        query_pt.emplace_back(queryV.row(query_i)(1));
        query_pt.emplace_back(queryV.row(query_i)(2));

        int _num_results = kdTree.index_->knnSearch(&query_pt[0], num_results + 1,
                                                    &ret_index[0],
                                                    &out_dist_sqr[0]);
        ret_index.resize(_num_results);
        out_dist_sqr.resize(_num_results);

        std::vector<Vector3d> neighPointList;
        for (size_t j = 0; j < _num_results; ++j) {
            if (ret_index[j] == query_i) continue;
            neighPointList.emplace_back(queryV.row(ret_index[j]));
            if (neighPointList.size() == num_results) break;
        }

        return neighPointList;
    }

    template<class KDTree>
    std::vector<std::vector<std::pair<Vector3d, double>>> getNeighborPoints(const KDTree &kdTree,
                                                                            const MatrixXd &queryV,
                                                                            const size_t &num_results) {
        int numQueries = queryV.rows();
        std::vector<std::vector<std::pair<Vector3d, double>>> neighPointList(numQueries);

#pragma omp parallel for
        for (int i = 0; i < numQueries; ++i) {
            // do a knn search
            std::vector<typename KDTree::IndexType> ret_index(num_results + 1);
            std::vector<double> out_dist_sqr(num_results + 1);

            std::vector<double> query_pt;
            query_pt.emplace_back(queryV.row(i)(0));
            query_pt.emplace_back(queryV.row(i)(1));
            query_pt.emplace_back(queryV.row(i)(2));

            int _num_results = kdTree.index_->knnSearch(&query_pt[0], num_results + 1,
                                                        &ret_index[0],
                                                        &out_dist_sqr[0]);

            // In case of less points in the tree than requested:
            ret_index.resize(_num_results);
            out_dist_sqr.resize(_num_results);

            for (size_t j = 0; j < _num_results; ++j) {
                if (ret_index[j] == i) continue;
                neighPointList[i].emplace_back(queryV.row(ret_index[j]), out_dist_sqr[j]);
                if (neighPointList[i].size() == num_results) break;
            }
        }

        return neighPointList;
    }

} // namespace psystem::knn_helper

#endif //PARTICLESYSTEM_KNNHELPER_H
