add_library(ContactGenerator STATIC
    ContactGeneratorBase.h
    MeshMeshContactGenerator.cpp
)
add_dependencies(ContactGenerator
    fcl
    CollisionObject
)
target_link_libraries(ContactGenerator
    OpenMP::OpenMP_CXX
    CollisionObject
    ${FCL_LIBRARY}
    ${CCD_LIBRARY}
)
