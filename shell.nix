{ pkgs ? import <nixpkgs> { } }:

pkgs.clangStdenv.mkDerivation (rec
{
  name="clang-shell";
  LD_LIBRARY_PATH = "${pkgs.clangStdenv.cc.cc.lib}/lib";
  nativeBuildInputs = (import ./deps.nix {inherit pkgs;}).shellDeps;

  shellHook = ''
    echo "Friction dev shell"
  '';
  CPATH = pkgs.lib.makeSearchPathOutput "dev" "include" (nativeBuildInputs);
})
