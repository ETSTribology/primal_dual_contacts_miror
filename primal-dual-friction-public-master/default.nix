{ pkgs ? import <nixpkgs> {} } :
let
    deps = (import ./deps.nix {inherit pkgs;});
in
pkgs.clangStdenv.mkDerivation {
    buildInputs = deps.buildDeps;
    nativeBuildInputs = deps.buildNativeDeps;
    cmakeFlags = ["-DBUILD_NIX_PACKAGE=ON"];
    name = "friction";
    src = ./.;
    configurePhase = ''
      cmake -DBUILD_NIX_PACKAGE=ON .
    '';
    buildPhase = ''
      make -j
      patchelf --force-rpath --set-rpath $ORIGIN/../../../libccd/lib64/ ./Release/lib/fcl/lib64/libfcl.so
      patchelf --force-rpath --set-rpath $ORIGIN/../../../libccd/lib64/ ./Release/lib/fcl/src/fcl-build/lib/libfcl.so
    '';
    installPhase = ''
      mkdir -p $out/lib
      cp -r ./Release/lib $out
      mkdir -p $out/bin
      cp ./Release/bin/ContactSimulation $out/bin
      cp ./Release/bin/CLIContactSimulation $out/bin
    '';
}
