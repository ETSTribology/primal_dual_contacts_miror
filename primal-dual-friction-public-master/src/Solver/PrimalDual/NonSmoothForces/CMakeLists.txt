add_subdirectory(ForceConstraint)
add_library(NonSmoothForces STATIC
    OnlyNormalForce.h
    NonSmoothContactForce.h
    NonSmoothForces.h
    NonSmoothForceBase.h
    NonSmoothContactForce.cpp
    OnlyNormalForce.cpp
)
target_link_libraries(NonSmoothForces
    OpenMP::OpenMP_CXX
    yaml-cpp::yaml-cpp
    Corrade::Utility
    CollisionObject
    Contact
	ForceConstraint
)
