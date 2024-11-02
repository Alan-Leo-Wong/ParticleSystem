//
// Created by Lei on 11/2/2023.
//

#include "src/ParticleSystem.h"

using namespace psystem;

int main(int argc, char **argv) {
    int numParticles = 500;
    int maxIter = 200;

    ParticleSystem particleSystem(argc > 1 ? argv[1] : "bunny.obj");
    particleSystem.printInfo();
    particleSystem.genInitParticles(numParticles, ParticleSystem::INIT_DISTRIBUTION::GAUSSIAN);
    particleSystem.outputParticles("init_particles.obj");
    particleSystem.run(maxIter, "opt_particles.obj");
    particleSystem.outputRDT("rdt.obj");

    return 0;
}