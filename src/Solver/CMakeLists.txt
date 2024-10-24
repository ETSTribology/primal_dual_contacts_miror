add_subdirectory(PrimalDual)
add_library(Solver STATIC Solver.h ForceSolverBase.h)
target_link_libraries(Solver PrimalDualSolver)
