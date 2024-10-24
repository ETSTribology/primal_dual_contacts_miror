{ pkgs ? import <nixpkgs> { } }:
let 
    deps = with pkgs; [
        glfw
        libzip
        xorg.libX11
    ];
    clangDeps = with pkgs; [
        clang-tools
        llvmPackages.openmp
    ];
    clangBuildDeps = with pkgs; [
        llvmPackages.openmp
    ];
    cmakeGuiDeps = with pkgs; [
        stdenv
        cmakeWithGui
    ];
    headerOnlyDeps = with pkgs; [
        eigen
        boost
    ];
    bDeps = with pkgs; [
        cmake
        patchelf
    ];
in
{
    shellDeps = cmakeGuiDeps ++ clangDeps ++ deps ++ headerOnlyDeps;
    buildDeps = deps;
    buildNativeDeps = bDeps ++ clangBuildDeps ++ headerOnlyDeps;
}
