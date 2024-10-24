set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

if(APPLICATION_USE_GUI)
    add_executable(ContactSimulation ContactSimulation.cpp)
    target_link_libraries(ContactSimulation PRIVATE
        Corrade::Main    
        Magnum::Application
        Magnum::GL
        Magnum::Magnum
        Magnum::SceneGraph
        MagnumIntegration::ImGui
        MagnumIntegration::Eigen
        ImGuizmo
        ImGuiFileDialog
        Cameras
        Drawables
        Dynamics
        IO
        )
endif()
add_executable(CLIContactSimulation CLIContactSimulation.cpp)
target_link_libraries(CLIContactSimulation PRIVATE
    Dynamics
    IO
)

# Make the executable a default target to build & run in Visual Studio
# set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT MyApplication)
