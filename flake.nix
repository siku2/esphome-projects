{
  description = "ESPHome components, packages and device configurations";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      treefmt-nix,
    }:
    let
      inherit (nixpkgs) lib;

      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];

      pythonFiles = lib.fileset.fileFilter (file: file.hasExt "py");

      perSystem = lib.genAttrs systems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
        in
        {
          inherit pkgs;
          treefmt = treefmt-nix.lib.evalModule pkgs ./treefmt.nix;
          validateConfigs = pkgs.writeShellApplication {
            name = "validate-configs";
            runtimeInputs = [
              pkgs.esphome
              pkgs.git
            ];
            text = ''
              cd "$(git rev-parse --show-toplevel)"
              if [ "$#" -eq 0 ]; then set -- tests/*.yaml; fi
              exec esphome config "$@"
            '';
          };
        }
      );

      forSystems = f: lib.mapAttrs (_: f) perSystem;
    in
    {
      formatter = forSystems ({ treefmt, ... }: treefmt.config.build.wrapper);

      checks = forSystems (
        { pkgs, treefmt, ... }:
        {
          formatting = treefmt.config.build.check self;

          # treefmt runs `ruff check --fix`, which reports what it cannot fix as
          # a formatting failure.
          lint-python =
            pkgs.runCommand "check-lint-python"
              {
                nativeBuildInputs = [ pkgs.ruff ];
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./ruff.toml
                    (pythonFiles ./components)
                    (pythonFiles ./projects)
                  ];
                };
              }
              ''
                ruff check --no-cache "$src"
                touch "$out"
              '';

          test-grill-cook =
            pkgs.runCommand "check-test-grill-cook"
              {
                nativeBuildInputs = [ pkgs.stdenv.cc ];
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./components/grill_cook/cook_model.h
                    ./components/grill_cook/tests/cook_model_test.cpp
                  ];
                };
              }
              ''
                g++ -std=c++17 -Wall -Wextra -Werror \
                  -o cook_model_test "$src/components/grill_cook/tests/cook_model_test.cpp"
                ./cook_model_test
                touch "$out"
              '';
        }
      );

      apps = forSystems (
        { validateConfigs, ... }:
        {
          validate-configs = {
            type = "app";
            program = lib.getExe validateConfigs;
            meta.description = "Run `esphome config` on every device configuration";
          };
        }
      );

      devShells = forSystems (
        {
          pkgs,
          treefmt,
          validateConfigs,
          ...
        }:
        {
          default = pkgs.mkShell {
            packages = [
              pkgs.esphome
              pkgs.clang-tools
              pkgs.ruff
              treefmt.config.build.wrapper
              validateConfigs
            ];
          };
        }
      );
    };
}
