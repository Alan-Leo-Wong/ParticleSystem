#include "ParticleSystem.h"
#include "KNNHelper.h"

#include <ostream>

#include <omp.h>
#include <LBFGS.h>
#include <igl/per_vertex_normals.h>
#include <igl/blue_noise.h>
#include <igl/doublearea.h>
#include <igl/PI.h>
#include <igl/principal_curvature.h>
#include <igl/read_triangle_mesh.h>


#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/numerics/predicates.h>
#include <geogram/voronoi/RVD.h>
#include <geogram/basic/command_line_args.h>

#ifdef ANISOTROPIC
#include <igl/principal_curvature.h>
#endif

namespace psystem {

    void check_for_zero_area_facets(GEO::Mesh &M) {
        using GEO::index_t;
        std::vector<unsigned int> remove_f;

        GEO::vec3 q1(0, 0, 0);
        GEO::vec3 q2(0, 0, 1);
        GEO::vec3 q3(0, 1, 0);
        GEO::vec3 q4(1, 0, 0);

        for (index_t f = 0; f < M.facets.nb(); ++f) {
            index_t c = M.facets.corners_begin(f);
            index_t v1 = M.facet_corners.vertex(c);
            index_t v2 = M.facet_corners.vertex(c + 1);
            index_t v3 = M.facet_corners.vertex(c + 2);
            const GEO::vec3 &p1 = GEO::Geom::mesh_vertex(M, v1);
            const GEO::vec3 &p2 = GEO::Geom::mesh_vertex(M, v2);
            const GEO::vec3 &p3 = GEO::Geom::mesh_vertex(M, v3);

            // Colinearity is tested by using four coplanarity
            // tests with points q1,q2,q3,q4 that are
            // not coplanar.
            if (GEO::PCK::orient_3d(p1, p2, p3, q1) == 0 &&
                GEO::PCK::orient_3d(p1, p2, p3, q2) == 0 &&
                GEO::PCK::orient_3d(p1, p2, p3, q3) == 0 &&
                GEO::PCK::orient_3d(p1, p2, p3, q4) == 0) {
                /*GEO::Logger::warn("Validate") << "[Particle System][Geogram]  Found a zero-area facet"
                                              << std::endl;*/
                remove_f.resize(M.facets.nb(), 0);
                remove_f[f] = 1;
            }
        }
        if (remove_f.size() != 0) {
            GEO::Logger::warn("Validate") << "[Particle System][Geogram]  Removing zero-area facet(s)"
                                          << std::endl;
            // M.facets.delete_elements(remove_f);
        }
    }

    ParticleSystem::ParticleSystem(const std::string &filename,
                                   bool _anisotropic)
            : in_file(filename), anisotropic(_anisotropic) {
        igl::read_triangle_mesh(filename, vertMat, faceMat);

        setData();
    }

    void ParticleSystem::setData() {
        igl::per_vertex_normals(vertMat, faceMat, igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE,
                                vertNormalMat);
        igl::per_face_normals(vertMat, faceMat, faceNormalMat);
        meshAABB.init(vertMat, faceMat);

        Vector3d boxMin = vertMat.colwise().minCoeff();
        Vector3d boxMax = vertMat.colwise().maxCoeff();
        sampleBox = AABox<Vector3d>(boxMin, boxMax);

        VectorXd dbla;
        igl::doublearea(vertMat, faceMat, dbla);
        Omega = 0.5 * dbla.sum();

        sigma = c_sigma * std::sqrt(Omega / (numParticles * 1.0)) /** 10*/;
        knn_numSearch = 5 * sigma;
        if (knn_numSearch == 0) knn_numSearch = 10;

        if (anisotropic) {
            unsigned radius = 5; // need to >= 2, default is 5
            igl::principal_curvature(vertMat, faceMat, vertPD_2, vertPD_1, vertPV_2, vertPV_1, radius);
        }
    }

    VectorXi ParticleSystem::genInitParticles(int _numParticles, const INIT_DISTRIBUTION &initDisType) {
        numParticles = _numParticles;
        if (numParticles <= 0) {
            std::cerr << "[Particle System]  The number of particles must be greater than zero!\n";
            exit(1);
        }

        std::vector<Vector3d> _particles;
        VectorXi blue_I;
        switch (initDisType) {
            case INIT_DISTRIBUTION::GAUSSIAN: {
#ifndef NDEBUG
                std::cout << "[Particle System]  Using Gaussian Sampling to generate initial particles.\n";
#endif

                pointgen::GaussianSampleGen<Vector3d> gsg(numParticles);
                _particles = gsg.run(sampleBox);

                particles = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor >>(
                        reinterpret_cast<double *>(_particles.data()), numParticles, 3);
            }
                break;
            case INIT_DISTRIBUTION::UNIFORM: {
#ifndef NDEBUG
                std::cout << "[Particle System]  Using Uniform Sampling to generate initial particles.\n";
#endif

                pointgen::UniformSampleGen<Vector3d> usg(numParticles);
                _particles = usg.run(sampleBox);
                particles = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor >>(
                        reinterpret_cast<double *>(_particles.data()), numParticles, 3);
            }
                break;
            case INIT_DISTRIBUTION::BLUE_NOISE:
                if (vertMat.rows() == 0 || faceMat.rows() == 0 || faceNormalMat.rows() == 0) {
                    std::cerr << "[Particle System]  Invalid input data in blue noise sampling.\n";
                    exit(1);
                }

#ifndef NDEBUG
                std::cout << "[Particle System]  Using Blue Noise Sampling to generate initial particles.\n";
#endif
                {
                    MatrixXd N_blue;
                    VectorXd A_blue;
                    MatrixXd C_blue;

                    // Heuristic to determine radius from desired number
                    const double r = [this](const int n) {
                        VectorXd A;
                        igl::doublearea(this->vertMat, this->faceMat, A);
                        return sqrt(((A.sum() * 0.5 / (n * 0.6162910373)) / igl::PI));
                    }(numParticles);
                    std::cout << "[Particle System]  Blue noise radius = " << r << std::endl;
                    MatrixXd B; // barycentric coordinates of face(from index in blue_I)
                    igl::blue_noise(vertMat, faceMat, r, B, blue_I, particles);
                }
                break;
            default:
                break;
        }

        return blue_I;
    }

    void ParticleSystem::genInitParticles(const std::string &particle_file) {
        Eigen::MatrixXi F;
        igl::read_triangle_mesh(particle_file, particles, F);
        numParticles = particles.rows();

        if (numParticles <= 0) {
            std::cerr << "[Particle System]  The number of particles must be greater than zero!\n";
            exit(1);
        }
    }

    void ParticleSystem::setSigmaByDis() {
        using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<
                Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>;

        // KNN search
        KDTree kdTree(3 /*dim*/, particles, {10 /* max leaf */ });
        std::vector<std::vector<std::pair<Vector3d, double>>> kd_neighPointList = knn_helper::getNeighborPoints(
                kdTree,
                particles,
                knn_numSearch);

        double points_dis = 0;
        for (int i = 0; i < numParticles; ++i) {
            double dis = .0;
            for (const auto &neighborPoint: kd_neighPointList[i]) {
                dis += sqrt(neighborPoint.second);
            }
            double avg_sigma = dis / (kd_neighPointList[i].size() * 1.0);
            points_dis += avg_sigma;
        }
        points_dis /= (numParticles + .0);
        sigma = c_sigma * points_dis;
    }

    void ParticleSystem::particleOptimize(int maxIter) {
        using namespace LBFGSpp;
        using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<MatrixXd>;

        auto optimize_fun = [&](Eigen::VectorXd &before_particle, Eigen::VectorXd &grad) {
            MatrixXd particleNormal(numParticles, 3);
            MatrixXd proj_particleMat(numParticles, 3);

#pragma omp parallel for
            for (int i = 0; i < numParticles; ++i) {
                VectorXd sqrD;
                VectorXi I;
                MatrixXd C;
                MatrixXd _i_particle(1, 3);
                _i_particle.row(0) = Vector3d(before_particle(i * 3),
                                              before_particle(i * 3 + 1),
                                              before_particle(i * 3 + 2));
                meshAABB.squared_distance(vertMat, faceMat, _i_particle, sqrD, I, C);
                Vector3d closest_point = C.row(0);
                Vector3d proj_normal = faceNormalMat.row(I(0));

                before_particle(i * 3) = closest_point(0);
                before_particle(i * 3 + 1) = closest_point(1);
                before_particle(i * 3 + 2) = closest_point(2);

                proj_particleMat.row(i) = closest_point;
                particleNormal.row(i) = proj_normal;
            }

            // KNN search
            KDTree kdTree(3 /*dim*/, proj_particleMat, {10 /* max leaf */ });
            std::vector<std::vector<std::pair<
                    Vector3d, double>>> kd_neighPointList = knn_helper::getNeighborPoints(
                    kdTree,
                    proj_particleMat,
                    knn_numSearch);

            double energy = .0;
//#pragma omp parallel for reduction(+:energy)
            for (int i = 0; i < numParticles; ++i) {
                const Vector3d i_particle = proj_particleMat.row(i);

                // Compute gradient of i'th particle
                Vector3d i_force;
                i_force.setZero();
                for (int j = 0; j < kd_neighPointList[i].size(); ++j) {
                    const Vector3d k_neiParticle = kd_neighPointList[i][j].first;

                    MatrixXd aniMetric(3, 3);
                    aniMetric.setIdentity();
                    // https://stackoverflow.com/questions/37492099/constructing-diagonal-matrix-in-eigen
                    /*Vector3d _I;
                    _I << 1, 1, 1;
                    aniMetric = _I.matrix().asDiagonal();*/
                    /// TODO: fix incorrect aniMetric
                    if (anisotropic) {
                        std::cout << "no!\n";
                        const Vector3d mid_ik = 0.5 * (i_particle + k_neiParticle);
                        MatrixXd ani_left(3, 3);
                        MatrixXd ani_diag(3, 3);

                        // Compute minimiun square distance
                        VectorXd sqrD;
                        VectorXi I;
                        MatrixXd C;
                        MatrixXd _mid_ik(1, 3);
                        _mid_ik.row(0) = mid_ik;
                        meshAABB.squared_distance(vertMat, faceMat, _mid_ik, sqrD, I, C);
                        Vector3d closest_point = C.row(0);

                        const Vector3i tri_vert_idx = faceMat.row(I(0));
                        const int vert_0 = tri_vert_idx(0);
                        const int vert_1 = tri_vert_idx(1);
                        const int vert_2 = tri_vert_idx(2);

                        Vector3d baryCoord;
                        // compute barycentric coordinates(fast but a bit inaccurate)
                        // http://gamedev.stackexchange.com/a/23745
                        {
                            const Vector3d tri_vert_0 = vertMat.row(vert_0);
                            const Vector3d tri_vert_1 = vertMat.row(vert_1);
                            const Vector3d tri_vert_2 = vertMat.row(vert_2);

                            Vector3d v0 = tri_vert_1 - tri_vert_0;
                            Vector3d v1 = tri_vert_2 - tri_vert_0;
                            Vector3d v2 = closest_point - tri_vert_0;
                            double d00 = v0.dot(v0);
                            double d01 = v0.dot(v1);
                            double d11 = v1.dot(v1);
                            double d20 = v2.dot(v0);
                            double d21 = v2.dot(v1);
                            double denom = d00 * d11 - d01 * d01;

                            baryCoord(2) = (d11 * d20 - d01 * d21) / denom;
                            baryCoord(1) = (d00 * d21 - d01 * d20) / denom;
                            baryCoord(0) = 1.0 - baryCoord(1) - baryCoord(2);
                        }

                        // compute midpoint's normal
                        Vector3d mid_n = baryCoord(0) * vertNormalMat.row(vert_0) +
                                         baryCoord(1) * vertNormalMat.row(vert_1) +
                                         baryCoord(2) * vertNormalMat.row(vert_2);

                        // compute midpoint's principal curvature
                        double PV_1 = baryCoord(0) * vertPV_1(vert_0) +
                                      baryCoord(1) * vertPV_1(vert_1) +
                                      baryCoord(2) * vertPV_1(vert_2);
                        double PV_2 = baryCoord(0) * vertPV_2(vert_0) +
                                      baryCoord(1) * vertPV_2(vert_1) +
                                      baryCoord(2) * vertPV_2(vert_2);

                        // update particle's principal curvature direction
                        Vector3d PD_1 = baryCoord(0) * vertPD_1.row(vert_0) +
                                        baryCoord(1) * vertPD_1.row(vert_1) +
                                        baryCoord(2) * vertPD_1.row(vert_2);
                        Vector3d PD_2 = baryCoord(0) * vertPD_2.row(vert_0) +
                                        baryCoord(1) * vertPD_2.row(vert_1) +
                                        baryCoord(2) * vertPD_2.row(vert_2);

                        ani_left.col(0) = PD_1/*.normalized()*/;
                        ani_left.col(1) = PD_2/*.normalized()*/;
                        ani_left.col(2) = mid_n.normalized();

                        Vector3d diag_val = Vector3d(PV_1, PV_2, 0);
                        ani_diag = diag_val.matrix().asDiagonal();

                        aniMetric = ani_left * ani_diag * (ani_left.transpose());

                        // aniMetric_Q = aniMetric.array().sqrt().matrix();
                    }

                    Vector3d v_ki = k_neiParticle - i_particle;
                    double t = (v_ki.transpose()) * aniMetric * v_ki;

                    double ik_energy = std::exp(-t / (4 * sigma * sigma));
                    if (std::isnan(ik_energy) || std::isinf(ik_energy)) {
                        std::cout << "[Particle System]  Error in setting anisotropic matrix.\n";
                        exit(1);
                    }
                    energy += ik_energy;

                    i_force += (aniMetric * v_ki / (2 * sigma * sigma)) * ik_energy;
                }

                // Project 'i_force' to the offset surface tangent
                Vector3d i_normal = particleNormal.row(i).normalized();
                i_force = i_force - (i_force.dot(i_normal)) * i_normal;

                // Update gradient
                grad(i * 3) = i_force.x();
                grad(i * 3 + 1) = i_force.y();
                grad(i * 3 + 2) = i_force.z();
            }

            return energy;
        };

        std::cout << "[Particle System]  Optimizing particle system by LBFGS.\n";
        // Solver's parameters
        LBFGSParam<double> param;
        param.epsilon = 1e-5;
        param.max_iterations = maxIter;

        // Create solver
        LBFGSSolver<double/*, LineSearchBracketing*/> solver(param);
        VectorXd particle_x = Eigen::Map<VectorXd>(particles.data(), numParticles * 3);

        int k;
        double systemEnergy;
        try {
            k = solver.minimize(optimize_fun, particle_x, systemEnergy);
            std::cout << "[Particle System]  Finished, system energy = " << systemEnergy << ".\n";
        } catch (const std::exception &e) {
            std::cout << "[Particle System]  Finished, system energy = " << systemEnergy << ".\n";
        }
        particles = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor >>(
                particle_x.data(),
                numParticles, 3);
    }

    void ParticleSystem::outputRDT(const std::string &out_file) {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
        GEO::CmdLine::import_arg_group("standard");
        GEO::CmdLine::import_arg_group("algo");

        GEO::Mesh M_in, points_in;
        GEO::mesh_load(in_file, M_in);
        GEO::mesh_load(particle_out_file, points_in);

        std::cout << "[Particle System]  Output mesh via RDT.\n";
        std::cout << "[Particle System][Geogram]  Check if mesh repair is needed.\n";
        GEO::mesh_repair(M_in);
        std::cout << "[Particle System][Geogram]  Check if the mesh contains zero facets.\n";
        check_for_zero_area_facets(M_in);

        GEO::Delaunay_var delaunay = GEO::Delaunay::create(3);
        GEO::RestrictedVoronoiDiagram_var RVD = GEO::RestrictedVoronoiDiagram::create(
                delaunay, &M_in
        );
        {
            double magnitude = 1e-7;
            double *p = points_in.vertices.point_ptr(0);
#pragma omp parallel for
            for (int i = 0; i < numParticles * 3; ++i) {
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis(-magnitude, magnitude);

                // Generate a small random perturbation for each component
                double perturbation = dis(gen);

                // Add perturbation to the vertex
                *(p + i) = *(p + i) + perturbation;
            }

//            GEO::Mesh sites_in;
//            sites_in.clear();
//            std::vector<double> sites(numParticles * 3);
//            for (int i = 0; i < numParticles; ++i) {
//                std::random_device rd;
//                std::mt19937 gen(rd());
//                std::uniform_real_distribution<> dis(-magnitude, magnitude);
//
//                // Generate a small random perturbation for each component
//                Eigen::RowVector3d perturbation(dis(gen), dis(gen), dis(gen));
//
//                // Add perturbation to the vertex
//                Eigen::RowVector3d v = particles.row(i) + perturbation;
//
//                /*GEO::index_t v_index = sites_in.vertices.create_vertex();
//                double *p = sites_in.vertices.point_ptr(v_index);
//                for (int c = 0; c < 3; ++c) {
//                    p[c] = v[c];
//                }*/
//
//                /*std::cout << v << std::endl;
//                sites[i * 3] = v(0);
//                sites[i * 3 + 1] = v(1);
//                sites[i * 3 + 2] = v(2);*/
//            }
            /*if(!sites_in.vertices.single_precision()) {
                GEO::index_t nb = sites_in.vertices.nb() * sites_in.vertices.dimension();
                double* p = sites_in.vertices.point_ptr(0);
                bool has_nan = false;
                for(int i = 0; i < nb; i++) {
                    if(GEO::Numeric::is_nan(*p)) {
                        has_nan = true;
                        *p = 0.0;
                    }
                    p++;
                }
                if(has_nan) {
                    GEO::Logger::warn("I/O") << "Found NaNs in input." << std::endl;
                }
            }*/

            /*delaunay->set_vertices(
                    numParticles, sites.data()
            );*/
            delaunay->set_vertices(
                    points_in.vertices.nb(), points_in.vertices.point_ptr(0)
            );
        }

        RVD->set_volumetric(false);

        {
            std::cout << "[Particle System][Geogram]  Computing RDT...\n";
            GEO::Mesh RDT;
            RVD->compute_RDT(RDT);
            GEO::MeshIOFlags flags;
            // flags.set_attribute(GEO::MESH_FACET_REGION);
            // flags.set_attribute(GEO::MESH_CELL_REGION);
            // flags.set_element(GEO::MESH_CELLS);
            flags.set_element(GEO::MESH_FACETS);
            GEO::mesh_save(RDT, out_file, flags);
        }
    }

    void ParticleSystem::run(int maxIter, const std::string &out_file) {
#ifndef NDEBUG
        printInfo();
#endif
        particleOptimize(maxIter);

        particle_out_file = out_file;
        outputParticles(out_file);
    }

    void ParticleSystem::outputParticles(const std::string &out_file) const {
        using namespace utils::file;

        checkDir(out_file);
        std::ofstream particleOut(out_file);
        if (!particleOut) {
            std::cerr << "I/O: File " << out_file << " could not be opened!\n";
            return;
        }

        std::cout << "[Particle System]  Output particles to " << std::quoted(out_file) << ".\n";
        if (getFileExtension(out_file) == ".obj")
            gvis::writePointCloud(particles, particleOut);
        else if (getFileExtension(out_file) == ".xyz")
            gvis::writePointCloud_xyz(particles, particleOut);
    }

    void ParticleSystem::printInfo() const {
        std::cout << "[Particle System]  The number of particles = " << numParticles << ".\n";
        std::cout << "[Particle System]  The number of search particles for KNN = " << knn_numSearch << ".\n";
        std::cout << "[Particle System]  Omega = " << Omega << ".\n";
        std::cout << "[Particle System]  sigma = " << sigma << ".\n";
    }

} // namespace psystem