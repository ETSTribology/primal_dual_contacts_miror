add_library(CollisionObject STATIC
    CollisionObject.h
    RigidObjectBase.cpp
    RigidObjectBase.h
    Mesh.cpp
    Mesh.h
    Cube.h
    Cube.cpp
    Sphere.cpp
    Sphere.h
    Ellipsoid.cpp
    Ellipsoid.h
    Convex.h
    Convex.cpp
)
add_dependencies(CollisionObject
    fcl
)
target_link_libraries(CollisionObject
    OpenMP::OpenMP_CXX
    Eigen3::Eigen
    MagnumIntegration::ImGui
    
    ${FCL_LIBRARY}
    ${CCD_LIBRARY}
)
