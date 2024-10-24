add_library(Cameras STATIC ArcBall.h ArcBallCamera.h OrthoCamera2D.h ArcBall.cpp)
target_link_libraries(Cameras Magnum::Magnum)
