//
// Created by Lei on 11/2/2023.
//

#include "src/ParticleSystem.h"

using namespace psystem;

struct Args {
    std::string inputPath;
    std::string outputPath;
    int numberOfParticles = 500;
    int maxIterations = 200;
};

bool parseArguments(int argc, char *argv[], Args &args) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-I") {
            if (i + 1 < argc) {
                args.inputPath = argv[++i];
            } else {
                std::cerr << "-I option requires one argument." << std::endl;
                return false;
            }
        } else if (arg == "-O") {
            if (i + 1 < argc) {
                args.outputPath = argv[++i];
            } else {
                std::cerr << "-O option requires one argument." << std::endl;
                return false;
            }
        } else if (arg == "-n") {
            if (i + 1 < argc) {
                args.numberOfParticles = std::atoi(argv[++i]);
                if (args.numberOfParticles <= 0) {
                    std::cerr << "-n option requires one argument and large than zero." << std::endl;
                    return false;
                }
            } else {
                std::cout << "warn: particle count not provided; using default of 500." << std::endl;
            }
        } else if (arg == "-m") {
            if (i + 1 < argc) {
                args.maxIterations = std::atoi(argv[++i]);
                if (args.maxIterations <= 0) {
                    std::cerr << "-m option requires one argument and large than zero." << std::endl;
                    return false;
                }
            } else {
                std::cout << "warn: maximum iterations not provided; using default of 500." << std::endl;
            }
        }
    }

    // Check if required options are provided
    if (args.inputPath.empty() || args.outputPath.empty()) {
        std::cerr << "All options must be specified: -I, -O, -n, -m" << std::endl;
        return false;
    }

    return true;
}

int main(int argc, char **argv) {
    Args args;
    if (!parseArguments(argc, argv, args)) {
        std::cerr << "Usage: " << argv[0]
                  << " -I <input_model_path> -O <output_path> -n <number_of_particles> -m <max_iterations>"
                  << std::endl;
        return 1;
    }

    ParticleSystem particleSystem(args.inputPath);
    particleSystem.printInfo();
    particleSystem.genInitParticles(args.numberOfParticles, ParticleSystem::INIT_DISTRIBUTION::GAUSSIAN);
    particleSystem.outputParticles("init_particles.obj");

    // optimizing particles
    particleSystem.run(args.maxIterations, "opt_particles.obj");

    // mesh generation via RDT
    particleSystem.outputRDT(args.outputPath);

    return 0;
}