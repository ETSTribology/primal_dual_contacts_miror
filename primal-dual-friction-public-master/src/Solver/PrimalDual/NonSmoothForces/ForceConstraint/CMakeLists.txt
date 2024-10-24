add_library(ForceConstraint STATIC
	ForceConstraint.h
	ForceConstraintBase.h
	LorentzCircleConstraint.h
	LorentzCircleConstraint.cpp
)
target_link_libraries(ForceConstraint
    Corrade::Utility
    CollisionObject
    Contact
)
