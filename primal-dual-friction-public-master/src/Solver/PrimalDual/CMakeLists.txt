add_subdirectory(NonSmoothForces)
add_library(PrimalDualSolver STATIC
    PrimalDualForceSolver.cpp
    PrimalDualForceSolver.h
)
target_link_libraries(PrimalDualSolver
    OpenMP::OpenMP_CXX
    Corrade::Utility
    CollisionObject
    Contact
    NonSmoothForces
)
