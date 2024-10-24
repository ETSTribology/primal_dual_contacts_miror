{
    description = "Le Groupe Wojtans Frolic of Frenetic Friction Frenzy";
    inputs = {
        nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
        flake-utils.url = "github:numtide/flake-utils";
    };
    outputs = {self, nixpkgs, flake-utils, ...}@inputs:
    flake-utils.lib.eachDefaultSystem (system:
        let
            pkgs = nixpkgs.legacyPackages.${system};
        in
        {
            devShells = {
                default = (import ./shell.nix {inherit pkgs;});
            };
            packages = {
                default = (import ./default.nix {inherit pkgs;});
            };
        }
    );
}
