//
// Created by lei on 11/11/2023.
//

#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include "Geometry.h"

#include <vector>
#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <igl/AABB.h>

#include <geogram/mesh/mesh.h>

namespace psystem {
    using namespace Eigen;

    class ParticleSystem {
    private:
        std::string in_file;
        std::string particle_out_file;
        bool anisotropic = false;
        int numParticles = 500;
        int knn_numSearch = 6;

        double c_sigma = 0.35;
        double sigma;
        double Omega;

    private:
        MatrixXd vertMat;
        MatrixXi faceMat;
        MatrixXd faceNormalMat;
        MatrixXd vertNormalMat;

        AABox<Vector3d> sampleBox;
        igl::AABB<MatrixXd, 3> meshAABB;

        VectorXd vertPV_1; // Minimum principal curvature
        VectorXd vertPV_2; // Maximum principal curvature
        MatrixXd vertPD_1; // Direction of minimum principal curvature
        MatrixXd vertPD_2; // Direction of maximum principal curvature

    public:
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> particles;
        std::vector<double> particleVec;

    public:
        enum class INIT_DISTRIBUTION {
            GAUSSIAN,
            UNIFORM,
            BLUE_NOISE
        };

    public:
        ParticleSystem() = default;

        ParticleSystem(const std::string &filename,
                       bool _anisotropic = false);

    private:
        void setSigmaByDis();

        void particleOptimize(int maxIter);

    public:
        VectorXi genInitParticles(int _numParticles, const INIT_DISTRIBUTION &initDisType = INIT_DISTRIBUTION::UNIFORM);

        void genInitParticles(const std::string &particle_file);

        MatrixXd getParticles() { return particles; }

        void setData();

        void printInfo() const;

        void outputParticles(const std::string &out_file) const;

        void outputRDT(const std::string &out_file);

        void run(int maxIter, const std::string &out_file);
    };

} // namespace psystem

#endif //PARTICLESYSTEM_H
