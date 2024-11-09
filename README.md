# Particle System

ParticleSystem is a CMake-based project that replicates the isotropic remeshing part of the paper ["Particle-Based Anisotropic Surface Meshing"](https://dl.acm.org/doi/10.1145/2461912.2461946). This implementation focuses on distributing a specified number of particles across the surface of an input mesh and optimizing their positions to achieve uniform distribution. The final high-quality mesh is generated using restricted Delaunay triangulation.

## Key Features

- **Particle Distribution**: Distribute particles uniformly over the surface of an input mesh.
- **Mesh Optimization**: Utilize the LBFGS optimization algorithm to minimize the energy function for particle positioning.
- **KD-Tree Construction**: Build KD-trees using nanoflann for efficient nearest neighbor queries.
- **Mesh Generation**: Generate the final mesh using restricted Delaunay triangulation.

## Prerequisites

Before setting up the ParticleSystem, ensure you have the following installed:
- CMake (version 3.21 or higher recommended)
- A C++ compiler that supports C++17
- Libraries: 
  - [jlblancoc/nanoflann: nanoflann: a C++11 header-only library for Nearest Neighbor (NN) search with KD-trees](https://github.com/jlblancoc/nanoflann)
  - [yixuan/LBFGSpp: A header-only C++ library for L-BFGS and L-BFGS-B algorithms](https://github.com/yixuan/LBFGSpp)
  - [BrunoLevy/geogram: a programming library with geometric algorithms](https://github.com/BrunoLevy/geogram) (for RDT)


## Getting Started

Clone the repository and build the project:

```bash
git clone https://github.com/Alan-Leo-Wong/ParticleSystem.git
cd ParticleSystem
mkdir build
cd build
cmake ..
cmake --build . -j your-core-num
```

## Usage

After building the project, you can run the executable using the following command format:

```
bash
./main -I <input_model_path> -O <output_path.obj> -n <number_of_particles> -m <max_iterations>
```

We have provided a bunny model in this repository.

### Example

To run the system with the provided bunny.obj model:

```
bash
./main -I bunny.obj -O ./output/bunny_remeshed.obj -n 1000 -m 100
```

This will distribute 1000 particles over the bunny model and perform up to 100 iterations to optimize their positions.