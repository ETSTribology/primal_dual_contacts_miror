# Primal-Dual Non-Smooth Friction for Rigid Body Animation
ACM SIGGRAPH 2024 Conference Papers

* [Project website](https://visualcomputing.ist.ac.at/publications/2024/PDNSF/)
* [Paper](https://pub.ista.ac.at/group_wojtan/projects/2024_Chen_FrictionFrenzy/sig24_friction_authors.pdf)
* [Video](https://pub.ista.ac.at/group_wojtan/projects/2024_Chen_FrictionFrenzy/friction_frenzy_main_video.mp4)

## Table of Contents
1. [Introduction](#introduction)
2. [Build](#how-to-build)
3. [Usage](#usage)
4. [Importing to Blender](#importing-simulation-results-to-blender-requires-blender--40)

## Introduction
This is the reference implementation for *Primal-Dual Non-Smooth Friction for Rigid Body Animation*.

The project consists of the following binaries after compilation:

* `ContactSimulation`: the main contact simulation program with GUI.
* `CLIContactSimulation`: a command-line only simulation program.

## Build
Building is tested on both `g++` and `clang++`.

### Dependencies
The programs require the following dependencies installed *on your system*:

* eigen >= 3.4.0
* glfw3
* libzip
* boost
* libX11

Additionally, the following libraries:

* [AMGCL](https://github.com/ddemidov/amgcl)
* [CNPY](https://github.com/rogersce/cnpy)
* [Flexible Collision Library](https://github.com/flexible-collision-library/fcl)
* [Dear ImGui](https://www.dearimgui.com/)
* [ImGuiFileDialog](https://github.com/aiekick/ImGuiFileDialog)
* [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo)
* [libccd](https://github.com/danfis/libccd)
* [libigl](https://libigl.github.io/)
* [Magnum](https://magnum.graphics)
	* corrade
	* magnum-intergration
	* magnum-plugins

are included as submodules in the repository. Download them with:
```
git submodule update --init --recursive
```
or, during cloning:
```
git clone --recursive https://github.com/ETSTribology/primal_dual_contacts_miror primal-dual
```

In particuar, we use a [custom fork](https://git.ista.ac.at/yichen/cnpy/-/tree/libzip-fixed) of CNPY which fixes its corrupting of NPZ files larger than 4GB (See [this issue](https://github.com/rogersce/cnpy/issues/39)).

From here on, there are three ways to build the package.

### CMake (Tested on Ubuntu 22.04 and NixOS)

1. Install the system dependencies. For Ubuntu users, run
```
sudo apt install cmake libeigen3-dev libglfw3-dev libzip-dev libboost-dev libx11-dev
```
The correct OpenMP implementation is also required for the project to compile with Clang. On Ubuntu 22.04 it is named `libomp-dev`.

2. Configure and build the project with CMake and make.
```
mkdir build
cd build
cmake ../
make -j
```

For NixOS users, the extra `BUILD_NIX_PACKAGE` flag must be set:
```
cmake -DBUILD_NIX_PACKAGE=ON ../
```

After compilation, the binaries should be found under `./Release/bin/`.

### Nix shell
You can also temporarily install all system dependencies using the [Nix package manager](https://nixos.org/). With Nix installed, simply go to the project root folder and run
```
nix-shell
```
or, if flakes are enabled:
```
nix develop
```

This creates a development shell in which all dependencies are installed. Now we can proceed with the build process:
```
mkdir build
cd build
cmake ../ #-DBUILD_NIX_PACKAGE still needed for NixOS!
make -j
```

> :information_source: NOTE
>
> Both Nix Shells and Nix builds still require initializing submodules!

### Nix build

You can also build the entire project as a Nix package by running from the project root:
```
nix-build
```
Nix will build and install the project under `./result/*`.

#### Using flakes
Nix flakes by default do not allow git submodules in packages. With the respective submodules pulled, run `nix build` with the following query to compile the nix package:
```
nix build ".?submodules=1"
```
This works with version 2.18.2, which is the latest stable version in nixpkgs as of June 2024.

> :warning: WARNING
>
> Starting with version 2.19, following query needs to be used instead (see [here](https://github.com/NixOS/nix/issues/9530)):
> ```
> nix build "git+file://$(pwd)?submodules=1"
> ```

> :x:
> **With version 2.20, the command above does not properly resolve git submodules either**. It is thus recommended to build with `nix-build` rather than with flakes. All newer versions of Nix are currently untested.

`BUILD_NIX_PACKAGE=ON` is automatically set in `default.nix`, and no further configuration should be required.

## Usage

If the project was build with nix, either with `nix-shell` or `nix-build` on a *non* NixOS distro, then the produced binaries further require NixGL to run. See [its official github repo](https://github.com/nix-community/nixGL) for setup instructions. Then, execute the binaries with
```
nixGL -- path/to/binaries/ContactSimulation
```
`ContactSimulation` requires no arguments. `CLIContactSimulation` receives two arguments:
```
path/to/binaries/CLIContactSimulation input.yaml output.gltf
```
Read the documentation in `scenes` for a detailed specifiction of the input files.

The output should consist of the following 4 files:
* `output.gltf`: a GLTF file of the scene, without animation.
* `output.bin`: the associated binary file of `output.gltf`
* `output.npz`: a zipped folder of all rigid object trajectories, stored as numpy arrays.
* `output.info`: a short summary of the simulation.

## Importing simulation results to Blender (requires Blender ≥ 4.0)
A script to import the simulation result into Blender is provided in `python/import_to_blender.py`. To use it, open the text editor in Blender and import the script. Click the play button and select the output GLTF file. *Make sure that the `.gltf`, `.bin`, and `.npz` share the same name and are in the same folder*.

The script will import the animation and adjust to the current frame rate. E.g. if a scene was simulated at 100 steps per second, and Blender is currently set to 60 fps, then the simulation will be reinterpolated to run at 60 fps. This saves some memory over importing an entire GLTF scene.

> :information_source: NOTE
> 
> Make sure that time stretching is not set under `Output > Format`, or the scene will not input properly!

![ball_primal_dual](https://github.com/user-attachments/assets/7ccc52d8-a65f-4780-8ddb-262938c03f0d)
![bolt_primal_dual](https://github.com/user-attachments/assets/97720b1a-125c-46fd-aac8-7920041a1144)


