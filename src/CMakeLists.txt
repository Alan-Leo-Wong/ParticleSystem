add_library(ParticleSystem STATIC Geometry.h KNNHelper.h ParticleSystem.h ParticleSystem.cpp)

target_link_libraries(ParticleSystem PUBLIC Eigen3::Eigen igl::core geogram::geogram)

target_include_directories(ParticleSystem PUBLIC ${CMAKE_SOURCE_DIR}/LBFGSpp/include)
target_include_directories(ParticleSystem PUBLIC ${CMAKE_SOURCE_DIR}/nanoflann/include)

find_package(OpenMP REQUIRED)
target_compile_options(ParticleSystem
        PUBLIC
        ${OpenMP_CXX_FLAGS})