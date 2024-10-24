add_library(Drawables STATIC
    ColoredDrawable.h
    ColoredDrawable.cpp
)
target_link_libraries(Drawables Magnum::Magnum CollisionObject)
